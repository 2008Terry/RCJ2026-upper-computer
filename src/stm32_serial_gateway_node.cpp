#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "rcj_localization/srv/stm32_command.hpp"

namespace
{

enum class CommandKind
{
  Distance,
  Turn,
  Request,
};

enum class ReplyStatus
{
  Ok,
  Eror,
};

struct Command
{
  CommandKind kind;
  double primary_value;
  double secondary_value;
};

struct ParsedReply
{
  std::string command_name;
  ReplyStatus status;
  double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
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
  }
  throw std::runtime_error("Unknown STM32 command kind.");
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
  return stream.str();
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

  throw std::runtime_error(
      "Unsupported STM32 command '" + command_name +
      "'. Supported commands: cmd_dis, cmd_turn, cmd_request");
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
    if (tokens >> reply.dx >> reply.dy >> reply.dtheta && !(tokens >> extra_token))
    {
      reply.status = ReplyStatus::Ok;
      return reply;
    }

    tokens.clear();
    tokens.str(command_text);
    if ((tokens >> reply.command_name >> status_text) && !(tokens >> extra_token) &&
        status_text == "eror")
    {
      reply.status = ReplyStatus::Eror;
      return reply;
    }

    return std::nullopt;
  }

  if (!(tokens >> status_text) || (tokens >> extra_token))
  {
    return std::nullopt;
  }

  if (status_text == "ok")
  {
    reply.status = ReplyStatus::Ok;
    return reply;
  }
  if (status_text == "eror")
  {
    reply.status = ReplyStatus::Eror;
    return reply;
  }

  return std::nullopt;
}

} // namespace

class Stm32SerialGatewayNode : public rclcpp::Node
{
public:
  using Stm32Command = rcj_localization::srv::Stm32Command;
  using Stm32CommandService = rclcpp::Service<Stm32Command>;

  Stm32SerialGatewayNode()
      : Node("stm32_serial_gateway_node")
  {
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter("baudrate", 115200);
    declare_parameter("tick_period_ms", 10);
    declare_parameter("resend_period_ms", 20);
    declare_parameter("command_timeout_ms", 50);
    declare_parameter("max_queue_size", 32);
    declare_parameter("enable_serial_log", true);
    declare_parameter("enable_raw_reply_log", false);

    port_ = get_parameter("port").as_string();
    baudrate_ = static_cast<int>(get_parameter("baudrate").as_int());
    tick_period_ms_ =
        std::max(1, static_cast<int>(get_parameter("tick_period_ms").as_int()));
    resend_period_ms_ =
        std::max(1, static_cast<int>(get_parameter("resend_period_ms").as_int()));
    command_timeout_ms_ =
        std::max(1, static_cast<int>(get_parameter("command_timeout_ms").as_int()));
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

    timer_ = create_wall_timer(
        std::chrono::milliseconds(tick_period_ms_),
        [this]()
        { tick(); });

    RCLCPP_INFO(
        get_logger(),
        "stm32_serial_gateway_node started. service='/stm32/send_command', port='%s', "
        "baudrate=%d, tick_period_ms=%d, resend_period_ms=%d, command_timeout_ms=%d, "
        "max_queue_size=%zu, enable_raw_reply_log=%s",
        port_.c_str(),
        baudrate_,
        tick_period_ms_,
        resend_period_ms_,
        command_timeout_ms_,
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
    std::uint32_t attempts = 0;
    bool started = false;
    bool awaiting_ack = false;
    bool write_in_progress = false;
    std::string outgoing_packet;
    std::size_t outgoing_offset = 0;
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

  void tick()
  {
    readIncomingData();

    if (!active_command_.has_value())
    {
      startNextCommand();
    }
    if (!active_command_.has_value())
    {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
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

    const bool should_send =
        !active_command_->awaiting_ack ||
        (now - active_command_->last_send_time) >= std::chrono::milliseconds(resend_period_ms_);
    if (should_send)
    {
      beginSendingActiveCommand(now);
    }
  }

  void startNextCommand()
  {
    if (pending_queue_.empty())
    {
      return;
    }

    active_command_ = std::move(pending_queue_.front());
    pending_queue_.pop_front();
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
    if (!active_command_.has_value())
    {
      return;
    }

    const auto parsed_reply = parseReplyLine(line);
    if (!parsed_reply.has_value())
    {
      return;
    }

    const std::string expected_command_name = commandName(active_command_->command.kind);
    if (parsed_reply->command_name != expected_command_name)
    {
      return;
    }

    if (parsed_reply->status == ReplyStatus::Ok)
    {
      if (enable_serial_log_)
      {
        if (active_command_->command.kind == CommandKind::Request)
        {
          RCLCPP_INFO(
              get_logger(),
              "STM32 replied to '%s' with dx=%.6f, dy=%.6f, dtheta=%.6f.",
              parsed_reply->command_name.c_str(),
              parsed_reply->dx,
              parsed_reply->dy,
              parsed_reply->dtheta);
        }
        else
        {
          RCLCPP_INFO(
              get_logger(),
              "STM32 acknowledged command '%s' with ok.",
              parsed_reply->command_name.c_str());
        }
      }

      if (active_command_->command.kind == CommandKind::Request)
      {
        std::ostringstream message;
        message << "STM32 request data received: dx=" << formatNumber(parsed_reply->dx)
                << ", dy=" << formatNumber(parsed_reply->dy)
                << ", dtheta=" << formatNumber(parsed_reply->dtheta) << ".";
        finishActiveCommand(
            true,
            "ok",
            message.str(),
            parsed_reply->dx,
            parsed_reply->dy,
            parsed_reply->dtheta);
      }
      else
      {
        finishActiveCommand(true, "ok", "STM32 command acknowledged.");
      }
      return;
    }

    if (enable_serial_log_)
    {
      RCLCPP_WARN(
          get_logger(),
          "STM32 replied 'eror' for '%s'; resending after %d ms unless command times out.",
          parsed_reply->command_name.c_str(),
          resend_period_ms_);
    }
  }

  void finishActiveCommand(
      bool success,
      const std::string &status,
      const std::string &message,
      double dx = 0.0,
      double dy = 0.0,
      double dtheta = 0.0)
  {
    if (!active_command_.has_value())
    {
      return;
    }

    const auto service = active_command_->service;
    const auto request_header = active_command_->request_header;
    const std::uint32_t attempts = active_command_->attempts;
    sendResponse(service, request_header, success, status, message, attempts, dx, dy, dtheta);
    active_command_.reset();
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
      double dtheta = 0.0)
  {
    Stm32Command::Response response;
    response.success = success;
    response.status = status;
    response.message = message;
    response.attempts = attempts;
    response.dx = dx;
    response.dy = dy;
    response.dtheta = dtheta;
    service->send_response(*request_header, response);
  }

  std::string port_;
  int baudrate_ = 115200;
  int tick_period_ms_ = 10;
  int resend_period_ms_ = 20;
  int command_timeout_ms_ = 50;
  std::size_t max_queue_size_ = 32;
  bool enable_serial_log_ = true;
  bool enable_raw_reply_log_ = false;
  int serial_fd_ = -1;
  std::string incoming_buffer_;
  std::deque<PendingCommand> pending_queue_;
  std::optional<PendingCommand> active_command_;
  rclcpp::Service<Stm32Command>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32SerialGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
