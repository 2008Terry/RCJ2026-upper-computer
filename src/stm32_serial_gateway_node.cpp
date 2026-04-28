#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rcj_localization/action/stm32_motion.hpp"
#include "rcj_localization/srv/stm32_command.hpp"

namespace
{

enum class CommandKind
{
  Distance,
  Turn,
  Request,
  Suck,
  Conmotion,
  Infred,
  InfredMode,
  Anglecal,
  McuReset,
};

enum class ReplyStatus
{
  Ok,
  Done,
  Busy,
  Eror,
};

struct Command
{
  CommandKind kind;
  double primary_value;
  double secondary_value;
  std::string text_value;
};

struct ParsedReply
{
  std::string command_name;
  std::string command_text;
  ReplyStatus status;
  int channel = 0;
  double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
  double theta = 0.0;
};

speed_t baudrateToSpeed(int baudrate)
{
  switch (baudrate)
  {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  default:
    throw std::runtime_error("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

std::uint16_t crc16CcittFalse(const std::string &data)
{
  std::uint16_t crc = 0xFFFF;
  for (unsigned char byte : data)
  {
    crc ^= static_cast<std::uint16_t>(byte << 8U);
    for (int bit = 0; bit < 8; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc = static_cast<std::uint16_t>(crc << 1U);
      }
    }
  }
  return crc;
}

std::string commandName(CommandKind kind)
{
  switch (kind)
  {
  case CommandKind::Distance:
    return "cmd_dis";
  case CommandKind::Turn:
    return "cmd_turn";
  case CommandKind::Request:
    return "cmd_request";
  case CommandKind::Suck:
    return "cmd_suck";
  case CommandKind::Conmotion:
    return "cmd_conmotion";
  case CommandKind::Infred:
    return "cmd_infred";
  case CommandKind::InfredMode:
    return "cmd_infred_mode";
  case CommandKind::Anglecal:
    return "cmd_anglecal";
  case CommandKind::McuReset:
    return "cmd_mcureset";
  }
  throw std::runtime_error("Unknown STM32 command kind.");
}

bool isMotionCommand(CommandKind kind)
{
  return kind == CommandKind::Distance || kind == CommandKind::Turn;
}

std::string formatNumber(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(15) << value;
  return stream.str();
}

std::string buildCommandText(const Command &command)
{
  std::ostringstream stream;
  stream << commandName(command.kind);
  if (command.kind == CommandKind::Distance)
  {
    stream << ' ' << formatNumber(command.primary_value)
           << ' ' << formatNumber(command.secondary_value);
  }
  else if (command.kind == CommandKind::Turn)
  {
    stream << ' ' << formatNumber(command.primary_value);
  }
  else if (command.kind == CommandKind::Suck ||
           command.kind == CommandKind::Conmotion)
  {
    stream << ' ' << static_cast<int>(command.primary_value);
  }
  else if (command.kind == CommandKind::InfredMode)
  {
    stream << ' ' << command.text_value;
  }
  return stream.str();
}

int parseIntegerToken(
    const std::string &token,
    const std::string &command_name,
    const std::string &spec)
{
  std::size_t consumed = 0;
  int value = 0;
  try
  {
    value = std::stoi(token, &consumed, 10);
  }
  catch (const std::exception &)
  {
    throw std::runtime_error(
        "Invalid " + command_name + " command '" + spec + "'. Expected an integer argument.");
  }

  if (consumed != token.size())
  {
    throw std::runtime_error(
        "Invalid " + command_name + " command '" + spec + "'. Expected an integer argument.");
  }

  return value;
}

std::optional<int> parseIntegerToken(const std::string &token)
{
  std::size_t consumed = 0;
  int value = 0;
  try
  {
    value = std::stoi(token, &consumed, 10);
  }
  catch (const std::exception &)
  {
    return std::nullopt;
  }

  if (consumed != token.size())
  {
    return std::nullopt;
  }

  return value;
}

Command parseCommandSpec(const std::string &spec)
{
  std::istringstream tokens(spec);
  std::string command_name;
  if (!(tokens >> command_name))
  {
    throw std::runtime_error("Empty STM32 command.");
  }

  Command command{};
  std::string extra_token;
  if (command_name == "cmd_dis")
  {
    command.kind = CommandKind::Distance;
    if (!(tokens >> command.primary_value >> command.secondary_value) ||
        (tokens >> extra_token))
    {
      throw std::runtime_error(
          "Invalid cmd_dis command '" + spec + "'. Expected: cmd_dis <primary> <secondary>");
    }
    return command;
  }

  if (command_name == "cmd_turn")
  {
    command.kind = CommandKind::Turn;
    if (!(tokens >> command.primary_value) || (tokens >> extra_token))
    {
      throw std::runtime_error(
          "Invalid cmd_turn command '" + spec + "'. Expected: cmd_turn <degrees>");
    }
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_request")
  {
    command.kind = CommandKind::Request;
    if (tokens >> extra_token)
    {
      throw std::runtime_error(
          "Invalid cmd_request command '" + spec + "'. Expected: cmd_request");
    }
    command.primary_value = 0.0;
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_suck")
  {
    command.kind = CommandKind::Suck;
    std::string speed_token;
    if (!(tokens >> speed_token) || (tokens >> extra_token))
    {
      throw std::runtime_error(
          "Invalid cmd_suck command '" + spec + "'. Expected: cmd_suck <speed 0-100>");
    }

    const int speed = parseIntegerToken(speed_token, command_name, spec);
    if (speed < 0 || speed > 100)
    {
      throw std::runtime_error(
          "Invalid cmd_suck command '" + spec + "'. Speed must be in range 0-100.");
    }
    command.primary_value = static_cast<double>(speed);
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_conmotion")
  {
    command.kind = CommandKind::Conmotion;
    std::string enabled_token;
    if (!(tokens >> enabled_token) || (tokens >> extra_token))
    {
      throw std::runtime_error(
          "Invalid cmd_conmotion command '" + spec + "'. Expected: cmd_conmotion <0|1>");
    }

    const int enabled = parseIntegerToken(enabled_token, command_name, spec);
    if (enabled != 0 && enabled != 1)
    {
      throw std::runtime_error(
          "Invalid cmd_conmotion command '" + spec + "'. Value must be 0 or 1.");
    }
    command.primary_value = static_cast<double>(enabled);
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_infred")
  {
    command.kind = CommandKind::Infred;
    if (tokens >> extra_token)
    {
      throw std::runtime_error(
          "Invalid cmd_infred command '" + spec + "'. Expected: cmd_infred");
    }
    command.primary_value = 0.0;
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_infred_mode")
  {
    command.kind = CommandKind::InfredMode;
    if (!(tokens >> command.text_value) || (tokens >> extra_token))
    {
      throw std::runtime_error(
          "Invalid cmd_infred_mode command '" + spec + "'. Expected: cmd_infred_mode <pt|tz>");
    }
    if (command.text_value != "pt" && command.text_value != "tz")
    {
      throw std::runtime_error(
          "Invalid cmd_infred_mode command '" + spec + "'. Mode must be 'pt' or 'tz'.");
    }
    command.primary_value = 0.0;
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_anglecal")
  {
    command.kind = CommandKind::Anglecal;
    if (tokens >> extra_token)
    {
      throw std::runtime_error(
          "Invalid cmd_anglecal command '" + spec + "'. Expected: cmd_anglecal");
    }
    command.primary_value = 0.0;
    command.secondary_value = 0.0;
    return command;
  }

  if (command_name == "cmd_mcureset")
  {
    command.kind = CommandKind::McuReset;
    if (tokens >> extra_token)
    {
      throw std::runtime_error(
          "Invalid cmd_mcureset command '" + spec + "'. Expected: cmd_mcureset");
    }
    command.primary_value = 0.0;
    command.secondary_value = 0.0;
    return command;
  }

  throw std::runtime_error(
      "Unsupported STM32 command '" + command_name +
      "'. Supported commands: cmd_dis, cmd_turn, cmd_request, cmd_suck, "
      "cmd_conmotion, cmd_infred, cmd_infred_mode, cmd_anglecal, cmd_mcureset");
}

std::string buildPacket(const Command &command)
{
  const std::string command_text = buildCommandText(command);
  std::ostringstream stream;
  stream << command_text << " *"
         << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
         << crc16CcittFalse(command_text) << "\r\n";
  return stream.str();
}

std::optional<ReplyStatus> parseReplyStatus(const std::string &status_text)
{
  if (status_text == "ok")
  {
    return ReplyStatus::Ok;
  }
  if (status_text == "done")
  {
    return ReplyStatus::Done;
  }
  if (status_text == "busy")
  {
    return ReplyStatus::Busy;
  }
  if (status_text == "eror")
  {
    return ReplyStatus::Eror;
  }
  return std::nullopt;
}

bool isSuccessStatus(ReplyStatus status)
{
  return status == ReplyStatus::Ok || status == ReplyStatus::Done;
}

std::string replyStatusText(ReplyStatus status)
{
  switch (status)
  {
  case ReplyStatus::Ok:
    return "ok";
  case ReplyStatus::Done:
    return "done";
  case ReplyStatus::Busy:
    return "busy";
  case ReplyStatus::Eror:
    return "eror";
  }
  throw std::runtime_error("Unknown STM32 reply status.");
}

std::optional<std::uint16_t> parseCrcHex(const std::string &crc_text)
{
  if (crc_text.empty() || crc_text.size() > 4)
  {
    return std::nullopt;
  }

  std::size_t consumed = 0;
  unsigned long value = 0;
  try
  {
    value = std::stoul(crc_text, &consumed, 16);
  }
  catch (const std::exception &)
  {
    return std::nullopt;
  }

  if (consumed != crc_text.size() ||
      value > std::numeric_limits<std::uint16_t>::max())
  {
    return std::nullopt;
  }

  return static_cast<std::uint16_t>(value);
}

std::string formatRawBytes(const char *data, std::size_t size)
{
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0');

  for (std::size_t i = 0; i < size; ++i)
  {
    const unsigned char byte = static_cast<unsigned char>(data[i]);
    if (byte == '\r')
    {
      stream << "\\r";
    }
    else if (byte == '\n')
    {
      stream << "\\n";
    }
    else if (byte >= 0x20U && byte <= 0x7EU)
    {
      stream << static_cast<char>(byte);
    }
    else
    {
      stream << "\\x" << std::setw(2) << static_cast<int>(byte);
    }
  }

  return stream.str();
}

std::optional<ParsedReply> parseReplyLine(const std::string &line)
{
  const std::size_t separator_pos = line.find('*');
  if (separator_pos == std::string::npos)
  {
    return std::nullopt;
  }

  std::string command_text = line.substr(0, separator_pos);
  while (!command_text.empty() && command_text.back() == ' ')
  {
    command_text.pop_back();
  }
  const std::string crc_text = line.substr(separator_pos + 1);
  const auto parsed_crc = parseCrcHex(crc_text);
  if (!parsed_crc.has_value() || crc16CcittFalse(command_text) != *parsed_crc)
  {
    return std::nullopt;
  }

  std::istringstream tokens(command_text);
  ParsedReply reply;
  if (!(tokens >> reply.command_name))
  {
    return std::nullopt;
  }

  std::string status_text;
  std::string extra_token;
  if (reply.command_name == "cmd_request")
  {
    if (tokens >> reply.dx >> reply.dy >> reply.dtheta >> reply.theta &&
        !(tokens >> extra_token))
    {
      reply.command_text = reply.command_name;
      reply.status = ReplyStatus::Ok;
      return reply;
    }

    tokens.clear();
    tokens.str(command_text);
    if ((tokens >> reply.command_name >> status_text) && !(tokens >> extra_token) &&
        status_text == "eror")
    {
      reply.command_text = reply.command_name;
      reply.status = ReplyStatus::Eror;
      return reply;
    }

    return std::nullopt;
  }

  if (reply.command_name == "cmd_dis")
  {
    Command command{};
    command.kind = CommandKind::Distance;
    if (tokens >> status_text && status_text == "done" &&
        tokens >> command.primary_value >> command.secondary_value &&
        !(tokens >> extra_token))
    {
      reply.command_text = buildCommandText(command);
      reply.status = ReplyStatus::Done;
      return reply;
    }

    tokens.clear();
    tokens.str(command_text);
    tokens >> reply.command_name;
    if (tokens >> command.primary_value >> command.secondary_value >> status_text &&
        !(tokens >> extra_token))
    {
      const auto status = parseReplyStatus(status_text);
      if (!status.has_value())
      {
        return std::nullopt;
      }
      reply.command_text = buildCommandText(command);
      reply.status = *status;
      return reply;
    }
  }
  else if (reply.command_name == "cmd_turn")
  {
    Command command{};
    command.kind = CommandKind::Turn;
    command.secondary_value = 0.0;
    if (tokens >> status_text && status_text == "done" &&
        tokens >> command.primary_value && !(tokens >> extra_token))
    {
      reply.command_text = buildCommandText(command);
      reply.status = ReplyStatus::Done;
      return reply;
    }

    tokens.clear();
    tokens.str(command_text);
    tokens >> reply.command_name;
    if (tokens >> command.primary_value >> status_text && !(tokens >> extra_token))
    {
      const auto status = parseReplyStatus(status_text);
      if (!status.has_value())
      {
        return std::nullopt;
      }
      reply.command_text = buildCommandText(command);
      reply.status = *status;
      return reply;
    }
  }
  else if (reply.command_name == "cmd_suck")
  {
    Command command{};
    command.kind = CommandKind::Suck;
    int speed = 0;
    if (tokens >> status_text >> speed && !(tokens >> extra_token))
    {
      const auto status = parseReplyStatus(status_text);
      if (!status.has_value())
      {
        return std::nullopt;
      }
      command.primary_value = static_cast<double>(speed);
      reply.command_text = buildCommandText(command);
      reply.status = *status;
      return reply;
    }
  }
  else if (reply.command_name == "cmd_conmotion")
  {
    Command command{};
    command.kind = CommandKind::Conmotion;
    int enabled = 0;
    if (tokens >> status_text >> enabled && !(tokens >> extra_token))
    {
      const auto status = parseReplyStatus(status_text);
      if (!status.has_value())
      {
        return std::nullopt;
      }
      command.primary_value = static_cast<double>(enabled);
      reply.command_text = buildCommandText(command);
      reply.status = *status;
      return reply;
    }
  }
  else if (reply.command_name == "cmd_infred")
  {
    std::string value_text;
    if (tokens >> value_text && !(tokens >> extra_token))
    {
      const auto channel = parseIntegerToken(value_text);
      if (channel.has_value() && *channel >= 1 && *channel <= 7)
      {
        reply.command_text = reply.command_name;
        reply.status = ReplyStatus::Ok;
        reply.channel = *channel;
        return reply;
      }
    }
  }
  else if (reply.command_name == "cmd_infred_mode")
  {
    Command command{};
    command.kind = CommandKind::InfredMode;
    if (tokens >> status_text >> command.text_value && !(tokens >> extra_token))
    {
      const auto status = parseReplyStatus(status_text);
      if (!status.has_value())
      {
        return std::nullopt;
      }
      reply.command_text = buildCommandText(command);
      reply.status = *status;
      return reply;
    }
  }
  else if (reply.command_name == "cmd_anglecal" ||
           reply.command_name == "cmd_mcureset")
  {
    if (tokens >> status_text && !(tokens >> extra_token))
    {
      const auto status = parseReplyStatus(status_text);
      if (!status.has_value())
      {
        return std::nullopt;
      }
      reply.command_text = reply.command_name;
      reply.status = *status;
      return reply;
    }
  }

  return std::nullopt;
}

} // namespace

class Stm32SerialGatewayNode : public rclcpp::Node
{
public:
  using Stm32Command = rcj_localization::srv::Stm32Command;
  using Stm32CommandService = rclcpp::Service<Stm32Command>;
  using Stm32Motion = rcj_localization::action::Stm32Motion;
  using GoalHandleStm32Motion = rclcpp_action::ServerGoalHandle<Stm32Motion>;

  Stm32SerialGatewayNode()
      : Node("stm32_serial_gateway_node")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<std::string>("motion_action_name", "/stm32/motion");
    declare_parameter("baudrate", 115200);
    declare_parameter("tick_period_ms", 10);
    declare_parameter("resend_period_ms", 20);
    declare_parameter("command_timeout_ms", 50);
    declare_parameter("motion_timeout_ms", 5000);
    declare_parameter("max_queue_size", 32);
    declare_parameter("enable_serial_log", true);
    declare_parameter("enable_raw_reply_log", false);

    port_ = get_parameter("port").as_string();
    motion_action_name_ = get_parameter("motion_action_name").as_string();
    if (motion_action_name_.empty())
    {
      throw std::runtime_error("Parameter 'motion_action_name' must not be empty.");
    }
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    tick_period_ms_ =
        std::max(1, static_cast<int>(get_parameter("tick_period_ms").as_int()));
    resend_period_ms_ =
        std::max(1, static_cast<int>(get_parameter("resend_period_ms").as_int()));
    command_timeout_ms_ =
        std::max(1, static_cast<int>(get_parameter("command_timeout_ms").as_int()));
    motion_timeout_ms_ =
        std::max(1, static_cast<int>(get_parameter("motion_timeout_ms").as_int()));
    max_queue_size_ =
        static_cast<std::size_t>(std::max(1, static_cast<int>(get_parameter("max_queue_size").as_int())));
    enable_serial_log_ = get_parameter("enable_serial_log").as_bool();
    enable_raw_reply_log_ = get_parameter("enable_raw_reply_log").as_bool();

    openAndConfigureSerial();

    service_ = create_service<Stm32Command>(
        "/stm32/send_command",
        [this](
            const std::shared_ptr<Stm32CommandService> service,
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<Stm32Command::Request> request)
        {
          handleCommandRequest(service, request_header, request);
        });

    motion_action_server_ = rclcpp_action::create_server<Stm32Motion>(
        this,
        motion_action_name_,
        std::bind(
            &Stm32SerialGatewayNode::handleMotionGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
        std::bind(
            &Stm32SerialGatewayNode::handleMotionCancel,
            this,
            std::placeholders::_1),
        std::bind(
            &Stm32SerialGatewayNode::handleMotionAccepted,
            this,
            std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(tick_period_ms_),
        [this]()
        { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "stm32_serial_gateway_node started. service='/stm32/send_command', motion_action='%s', port='%s', "
        "baudrate=%d, tick_period_ms=%d, resend_period_ms=%d(no-op), command_timeout_ms=%d, "
        "motion_timeout_ms=%d, max_queue_size=%zu, enable_raw_reply_log=%s",
        motion_action_name_.c_str(),
        port_.c_str(),
        baudrate_,
        tick_period_ms_,
        resend_period_ms_,
        command_timeout_ms_,
        motion_timeout_ms_,
        max_queue_size_,
        enable_raw_reply_log_ ? "true" : "false");
  }

  ~Stm32SerialGatewayNode() override
  {
    closeSerial();
  }

private:
  struct PendingCommand
  {
    Command command;
    std::string command_text;
    std::shared_ptr<Stm32CommandService> service;
    std::shared_ptr<rmw_request_id_t> request_header;
    std::shared_ptr<GoalHandleStm32Motion> motion_goal_handle;
    std::uint32_t attempts = 0;
    bool started = false;
    bool awaiting_ack = false;
    bool write_in_progress = false;
    std::string outgoing_packet;
    std::size_t outgoing_offset = 0;
    std::chrono::steady_clock::time_point enqueue_time{};
    std::chrono::steady_clock::time_point start_time{};
    std::chrono::steady_clock::time_point last_send_time{};
  };

  void handleCommandRequest(
      const std::shared_ptr<Stm32CommandService> &service,
      const std::shared_ptr<rmw_request_id_t> &request_header,
      const std::shared_ptr<Stm32Command::Request> &request)
  {
    Command command{};
    try
    {
      command = parseCommandSpec(request->command);
    }
    catch (const std::exception &error)
    {
      sendResponse(service, request_header, false, "invalid_command", error.what(), 0);
      return;
    }

    if (isMotionCommand(command.kind))
    {
      sendResponse(
          service,
          request_header,
          false,
          "unsupported_command",
          "cmd_dis and cmd_turn are available through the /stm32/motion action, not /stm32/send_command.",
          0);
      return;
    }

    if (pending_queue_.size() >= max_queue_size_)
    {
      sendResponse(
          service,
          request_header,
          false,
          "queue_full",
          "STM32 command queue is full.",
          0);
      return;
    }

    PendingCommand pending;
    pending.command = command;
    pending.command_text = buildCommandText(command);
    pending.service = service;
    pending.request_header = request_header;
    pending.enqueue_time = std::chrono::steady_clock::now();
    pending_queue_.push_back(std::move(pending));

    if (enable_serial_log_)
    {
      RCLCPP_INFO(
          get_logger(),
          "Queued STM32 command: %s (queue_size=%zu)",
          pending_queue_.back().command_text.c_str(),
          pending_queue_.size());
    }
  }

  rclcpp_action::GoalResponse handleMotionGoal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Stm32Motion::Goal> goal)
  {
    try
    {
      const Command command = parseCommandSpec(goal->command);
      if (!isMotionCommand(command.kind))
      {
        RCLCPP_WARN(
            get_logger(),
            "Rejected STM32 motion action goal '%s': expected cmd_dis or cmd_turn.",
            goal->command.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN(
          get_logger(),
          "Rejected invalid STM32 motion action goal '%s': %s",
          goal->command.c_str(),
          error.what());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleMotionCancel(
      const std::shared_ptr<GoalHandleStm32Motion>)
  {
    RCLCPP_WARN(
        get_logger(),
        "Rejecting STM32 motion action cancel request: STM32 cancel is not implemented.");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handleMotionAccepted(
      const std::shared_ptr<GoalHandleStm32Motion> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    Command command{};
    try
    {
      command = parseCommandSpec(goal->command);
      if (!isMotionCommand(command.kind))
      {
        sendMotionResult(
            goal_handle,
            false,
            "invalid_command",
            "STM32 motion action only accepts cmd_dis and cmd_turn.",
            0);
        return;
      }
    }
    catch (const std::exception &error)
    {
      sendMotionResult(goal_handle, false, "invalid_command", error.what(), 0);
      return;
    }

    if (pending_queue_.size() >= max_queue_size_)
    {
      sendMotionResult(
          goal_handle,
          false,
          "queue_full",
          "STM32 command queue is full.",
          0);
      return;
    }

    PendingCommand pending;
    pending.command = command;
    pending.command_text = buildCommandText(command);
    pending.motion_goal_handle = goal_handle;
    pending.enqueue_time = std::chrono::steady_clock::now();
    pending_queue_.push_back(std::move(pending));

    if (enable_serial_log_)
    {
      RCLCPP_INFO(
          get_logger(),
          "Queued STM32 motion action: %s (queue_size=%zu)",
          pending_queue_.back().command_text.c_str(),
          pending_queue_.size());
    }
  }

  void tick()
  {
    readIncomingData();

    const auto now = std::chrono::steady_clock::now();
    expireQueuedRequestCommands(now);
    expireTimedOutMotionCommand(now);

    if (!active_command_.has_value())
    {
      startNextCommand();
    }
    if (!active_command_.has_value())
    {
      return;
    }

    if (active_command_->started &&
        (now - active_command_->start_time) >= std::chrono::milliseconds(command_timeout_ms_))
    {
      const std::string message =
          "STM32 command timed out after " + std::to_string(command_timeout_ms_) + " ms.";
      finishActiveCommand(false, "timeout", message);
      startNextCommand();
      return;
    }

    if (active_command_->write_in_progress)
    {
      continueWritingActiveCommand();
      return;
    }

    if (!active_command_->started)
    {
      beginSendingActiveCommand(now);
    }
  }

  void expireQueuedRequestCommands(const std::chrono::steady_clock::time_point &now)
  {
    auto command = pending_queue_.begin();
    while (command != pending_queue_.end())
    {
      if (command->command.kind != CommandKind::Request ||
          command->enqueue_time == std::chrono::steady_clock::time_point{} ||
          (now - command->enqueue_time) < std::chrono::milliseconds(command_timeout_ms_))
      {
        ++command;
        continue;
      }

      const std::string message =
          "STM32 cmd_request timed out in gateway queue after " +
          std::to_string(command_timeout_ms_) + " ms before it was sent.";
      if (enable_serial_log_)
      {
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
      }
      sendCommandResult(*command, false, "timeout", message);
      command = pending_queue_.erase(command);
    }
  }

  void expireTimedOutMotionCommand(const std::chrono::steady_clock::time_point &now)
  {
    if (!active_motion_command_.has_value() || !active_motion_command_->started)
    {
      return;
    }

    if ((now - active_motion_command_->start_time) < std::chrono::milliseconds(motion_timeout_ms_))
    {
      return;
    }

    const std::string command_text = active_motion_command_->command_text;
    const std::string message =
        "STM32 motion command '" + command_text + "' timed out after " +
        std::to_string(motion_timeout_ms_) + " ms waiting for completion ACK.";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    finishActiveMotionCommand(false, "motion_timeout", message);
  }

  void startNextCommand()
  {
    if (pending_queue_.empty())
    {
      return;
    }

    auto next_command = pending_queue_.begin();
    if (active_motion_command_.has_value())
    {
      next_command = std::find_if(
          pending_queue_.begin(),
          pending_queue_.end(),
          [](const PendingCommand &pending)
          {
            return pending.command.kind == CommandKind::Request;
          });
      if (next_command == pending_queue_.end())
      {
        return;
      }
    }

    active_command_ = std::move(*next_command);
    pending_queue_.erase(next_command);
  }

  void beginSendingActiveCommand(const std::chrono::steady_clock::time_point &now)
  {
    if (!active_command_.has_value())
    {
      return;
    }

    auto &command = *active_command_;
    if (!command.started)
    {
      command.started = true;
      command.start_time = now;
    }

    command.outgoing_packet = buildPacket(command.command);
    command.outgoing_offset = 0;
    command.write_in_progress = true;
    command.awaiting_ack = false;
    command.last_send_time = now;
    ++command.attempts;
    continueWritingActiveCommand();
  }

  void continueWritingActiveCommand()
  {
    if (!active_command_.has_value())
    {
      return;
    }

    auto &command = *active_command_;
    while (command.outgoing_offset < command.outgoing_packet.size())
    {
      const char *data = command.outgoing_packet.data() + command.outgoing_offset;
      const std::size_t remaining = command.outgoing_packet.size() - command.outgoing_offset;
      const ssize_t bytes_written = ::write(serial_fd_, data, remaining);

      if (bytes_written > 0)
      {
        command.outgoing_offset += static_cast<std::size_t>(bytes_written);
        continue;
      }

      if (bytes_written == 0)
      {
        return;
      }

      if (errno == EINTR)
      {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return;
      }

      const std::string message =
          "Serial write failed: " + std::string(std::strerror(errno));
      finishActiveCommand(false, "write_error", message);
      startNextCommand();
      return;
    }

    const std::string sent_packet = command.outgoing_packet;
    command.outgoing_packet.clear();
    command.outgoing_offset = 0;
    command.write_in_progress = false;
    command.awaiting_ack = true;

    if (enable_serial_log_ && sent_packet.size() >= 2)
    {
      RCLCPP_INFO(
          get_logger(),
          "Sent STM32 command: %s",
          sent_packet.substr(0, sent_packet.size() - 2).c_str());
    }

    if (isMotionCommand(command.command.kind))
    {
      active_motion_command_ = std::move(command);
      active_command_.reset();
    }
  }

  void openAndConfigureSerial()
  {
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      throw std::runtime_error(
          "Failed to open serial port '" + port_ + "': " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      const std::string error = std::strerror(errno);
      closeSerial();
      throw std::runtime_error("tcgetattr failed: " + error);
    }

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CLOCAL | CREAD;

    const speed_t speed = baudrateToSpeed(baudrate_);
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0)
    {
      const std::string error = std::strerror(errno);
      closeSerial();
      throw std::runtime_error("Failed to configure baudrate: " + error);
    }

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      const std::string error = std::strerror(errno);
      closeSerial();
      throw std::runtime_error("tcsetattr failed: " + error);
    }

    const int flags = fcntl(serial_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(serial_fd_, F_SETFL, flags | O_NONBLOCK) != 0)
    {
      const std::string error = std::strerror(errno);
      closeSerial();
      throw std::runtime_error("Failed to set non-blocking serial mode: " + error);
    }

    tcflush(serial_fd_, TCIOFLUSH);
  }

  void closeSerial()
  {
    if (serial_fd_ >= 0)
    {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void readIncomingData()
  {
    char buffer[256];

    while (true)
    {
      const ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
      if (bytes_read > 0)
      {
        if (enable_raw_reply_log_)
        {
          RCLCPP_INFO(
              get_logger(),
              "Received STM32 raw chunk: %s",
              formatRawBytes(buffer, static_cast<std::size_t>(bytes_read)).c_str());
        }
        incoming_buffer_.append(buffer, static_cast<std::size_t>(bytes_read));
        processBufferedLines();
        continue;
      }

      if (bytes_read == 0)
      {
        return;
      }

      if (errno == EINTR)
      {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return;
      }

      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Serial read failed: %s",
          std::strerror(errno));
      return;
    }
  }

  void processBufferedLines()
  {
    while (true)
    {
      const std::size_t newline_pos = incoming_buffer_.find('\n');
      if (newline_pos == std::string::npos)
      {
        return;
      }

      std::string line = incoming_buffer_.substr(0, newline_pos);
      incoming_buffer_.erase(0, newline_pos + 1);
      if (!line.empty() && line.back() == '\r')
      {
        line.pop_back();
      }
      if (line.empty())
      {
        continue;
      }

      if (enable_raw_reply_log_)
      {
        RCLCPP_INFO(get_logger(), "Received STM32 raw reply line: %s", line.c_str());
      }

      handleReplyLine(line);
    }
  }

  void handleReplyLine(const std::string &line)
  {
    const auto parsed_reply = parseReplyLine(line);
    if (!parsed_reply.has_value())
    {
      return;
    }

    if (active_command_.has_value() &&
        replyMatchesCommand(*parsed_reply, *active_command_))
    {
      handleActiveCommandReply(*parsed_reply);
      return;
    }

    if (active_motion_command_.has_value() &&
        replyMatchesMotionCompletion(*parsed_reply, *active_motion_command_))
    {
      handleActiveMotionReply(*parsed_reply);
      return;
    }
  }

  bool replyMatchesCommand(
      const ParsedReply &parsed_reply,
      const PendingCommand &pending) const
  {
    const std::string expected_command_name = commandName(pending.command.kind);
    if (parsed_reply.command_name != expected_command_name)
    {
      return false;
    }
    return parsed_reply.command_text == pending.command_text ||
           parsed_reply.command_text == expected_command_name;
  }

  bool replyMatchesMotionCompletion(
      const ParsedReply &parsed_reply,
      const PendingCommand &pending) const
  {
    const std::string expected_command_name = commandName(pending.command.kind);
    if (parsed_reply.command_name != expected_command_name)
    {
      return false;
    }

    // Motion actions must complete only on the full command ACK. A generic
    // "cmd_dis ok" / "cmd_turn ok" can only prove the STM32 parsed the command,
    // not that the movement finished.
    return parsed_reply.command_text == pending.command_text;
  }

  void handleActiveCommandReply(const ParsedReply &parsed_reply)
  {
    if (isSuccessStatus(parsed_reply.status))
    {
      if (enable_serial_log_)
      {
        if (active_command_->command.kind == CommandKind::Request)
        {
          RCLCPP_INFO(
              get_logger(),
              "STM32 replied to '%s' with dx=%.6f, dy=%.6f, dtheta=%.6f, theta=%.6f.",
              parsed_reply.command_name.c_str(),
              parsed_reply.dx,
              parsed_reply.dy,
              parsed_reply.dtheta,
              parsed_reply.theta);
        }
        else if (active_command_->command.kind == CommandKind::Infred &&
                 parsed_reply.channel != 0)
        {
          RCLCPP_INFO(
              get_logger(),
              "STM32 replied to '%s' with channel=%d.",
              parsed_reply.command_name.c_str(),
              parsed_reply.channel);
        }
        else
        {
          RCLCPP_INFO(
              get_logger(),
              "STM32 acknowledged command '%s' with %s.",
              active_command_->command_text.c_str(),
              replyStatusText(parsed_reply.status).c_str());
        }
      }

      if (active_command_->command.kind == CommandKind::Request)
      {
        std::ostringstream message;
        message << "STM32 request data received: dx=" << formatNumber(parsed_reply.dx)
                << ", dy=" << formatNumber(parsed_reply.dy)
                << ", dtheta=" << formatNumber(parsed_reply.dtheta)
                << ", theta=" << formatNumber(parsed_reply.theta) << ".";
        finishActiveCommand(
            true,
            "ok",
            message.str(),
            parsed_reply.dx,
            parsed_reply.dy,
            parsed_reply.dtheta,
            parsed_reply.theta);
      }
      else if (active_command_->command.kind == CommandKind::Infred &&
               parsed_reply.channel != 0)
      {
        finishActiveCommand(
            true,
            "ok",
            "STM32 infrared channel received: channel=" +
                std::to_string(parsed_reply.channel) + ".");
      }
      else
      {
        finishActiveCommand(
            true,
            replyStatusText(parsed_reply.status),
            "STM32 command acknowledged.");
      }
      return;
    }

    if (enable_serial_log_)
    {
      RCLCPP_WARN(
          get_logger(),
          "STM32 replied '%s' for '%s'.",
          replyStatusText(parsed_reply.status).c_str(),
          parsed_reply.command_name.c_str());
    }
    finishActiveCommand(
        false,
        replyStatusText(parsed_reply.status),
        "STM32 replied " + replyStatusText(parsed_reply.status) + ".");
  }

  void handleActiveMotionReply(const ParsedReply &parsed_reply)
  {
    if (!active_motion_command_.has_value())
    {
      return;
    }

    if (isSuccessStatus(parsed_reply.status))
    {
      if (enable_serial_log_)
      {
        RCLCPP_INFO(
            get_logger(),
            "STM32 acknowledged command '%s' with %s.",
            active_motion_command_->command_text.c_str(),
            replyStatusText(parsed_reply.status).c_str());
      }
      finishActiveMotionCommand(
          true,
          replyStatusText(parsed_reply.status),
          "STM32 command acknowledged.");
      return;
    }

    if (enable_serial_log_)
    {
      RCLCPP_WARN(
          get_logger(),
          "STM32 replied '%s' for active motion command '%s'.",
          replyStatusText(parsed_reply.status).c_str(),
          active_motion_command_->command_text.c_str());
    }
    finishActiveMotionCommand(
        false,
        replyStatusText(parsed_reply.status),
        "STM32 replied " + replyStatusText(parsed_reply.status) +
            " for active motion command.");
  }

  void finishActiveCommand(
      bool success,
      const std::string &status,
      const std::string &message,
      double dx = 0.0,
      double dy = 0.0,
      double dtheta = 0.0,
      double theta = 0.0)
  {
    if (!active_command_.has_value())
    {
      return;
    }

    const PendingCommand completed_command = *active_command_;
    sendCommandResult(completed_command, success, status, message, dx, dy,
                      dtheta, theta);
    active_command_.reset();
  }

  void finishActiveMotionCommand(
      bool success,
      const std::string &status,
      const std::string &message)
  {
    if (!active_motion_command_.has_value())
    {
      return;
    }

    const PendingCommand completed_command = *active_motion_command_;
    sendCommandResult(completed_command, success, status, message);
    active_motion_command_.reset();
  }

  void sendCommandResult(
      const PendingCommand &command,
      bool success,
      const std::string &status,
      const std::string &message,
      double dx = 0.0,
      double dy = 0.0,
      double dtheta = 0.0,
      double theta = 0.0)
  {
    if (command.motion_goal_handle)
    {
      sendMotionResult(
          command.motion_goal_handle,
          success,
          status,
          message,
          command.attempts);
      return;
    }

    sendResponse(
        command.service,
        command.request_header,
        success,
        status,
        message,
        command.attempts,
        dx,
        dy,
        dtheta,
        theta);
  }

  void sendMotionResult(
      const std::shared_ptr<GoalHandleStm32Motion> &goal_handle,
      bool success,
      const std::string &status,
      const std::string &message,
      std::uint32_t attempts)
  {
    auto result = std::make_shared<Stm32Motion::Result>();
    result->success = success;
    result->status = status;
    result->message = message;
    result->attempts = attempts;

    if (success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
  }

  void sendResponse(
      const std::shared_ptr<Stm32CommandService> &service,
      const std::shared_ptr<rmw_request_id_t> &request_header,
      bool success,
      const std::string &status,
      const std::string &message,
      std::uint32_t attempts,
      double dx = 0.0,
      double dy = 0.0,
      double dtheta = 0.0,
      double theta = 0.0)
  {
    Stm32Command::Response response;
    response.success = success;
    response.status = status;
    response.message = message;
    response.attempts = attempts;
    response.dx = dx;
    response.dy = dy;
    response.dtheta = dtheta;
    response.theta = theta;
    service->send_response(*request_header, response);
  }

  std::string port_;
  std::string motion_action_name_;
  int baudrate_ = 115200;
  int tick_period_ms_ = 10;
  int resend_period_ms_ = 20;
  int command_timeout_ms_ = 50;
  int motion_timeout_ms_ = 5000;
  std::size_t max_queue_size_ = 32;
  bool enable_serial_log_ = true;
  bool enable_raw_reply_log_ = false;
  int serial_fd_ = -1;
  std::string incoming_buffer_;
  std::deque<PendingCommand> pending_queue_;
  std::optional<PendingCommand> active_command_;
  std::optional<PendingCommand> active_motion_command_;
  rclcpp::Service<Stm32Command>::SharedPtr service_;
  rclcpp_action::Server<Stm32Motion>::SharedPtr motion_action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32SerialGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
