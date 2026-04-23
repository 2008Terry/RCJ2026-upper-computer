#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace
{

  enum class CommandKind
  {
    Distance,
    Turn,
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
    return kind == CommandKind::Distance ? "cmd_dis" : "cmd_turn";
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
    else
    {
      stream << ' ' << formatNumber(command.primary_value);
    }
    return stream.str();
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
    std::string status_text;
    std::string extra_token;
    if (!(tokens >> reply.command_name >> status_text) || (tokens >> extra_token))
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

class Stm32BehaviorNode : public rclcpp::Node
{
public:
  Stm32BehaviorNode()
      : Node("stm32_behavior_node"),
        command_sequence_{
            {CommandKind::Distance, 100.0, -100.0},
            {CommandKind::Turn, 90.0, 0.0},
            {CommandKind::Distance, 50.0, 0.0},
        }
  {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("timeout_sec", 0.05);
    this->declare_parameter("resend_period_ms", 200);
    this->declare_parameter("tick_period_ms", 20);
    this->declare_parameter("enable_serial_log", true);
    this->declare_parameter("enable_raw_reply_log", false);

    port_ = this->get_parameter("port").as_string();
    baudrate_ = static_cast<int>(this->get_parameter("baudrate").as_int());
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();
    resend_period_ms_ =
        std::max(1, static_cast<int>(this->get_parameter("resend_period_ms").as_int()));
    tick_period_ms_ =
        std::max(1, static_cast<int>(this->get_parameter("tick_period_ms").as_int()));
    enable_serial_log_ = this->get_parameter("enable_serial_log").as_bool();
    enable_raw_reply_log_ = this->get_parameter("enable_raw_reply_log").as_bool();

    openAndConfigureSerial();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tick_period_ms_),
        std::bind(&Stm32BehaviorNode::tick, this));

    RCLCPP_INFO(
        this->get_logger(),
        "stm32_behavior_node started. port='%s', baudrate=%d, timeout_sec=%.3f, "
        "resend_period_ms=%d, tick_period_ms=%d, enable_raw_reply_log=%s",
        port_.c_str(),
        baudrate_,
        timeout_sec_,
        resend_period_ms_,
        tick_period_ms_,
        enable_raw_reply_log_ ? "true" : "false");
  }

  ~Stm32BehaviorNode() override
  {
    closeSerial();
  }

private:
  const Command &currentCommand() const
  {
    return command_sequence_.at(current_command_index_);
  }

  void tick()
  {
    if (completed_)
    {
      return;
    }

    readIncomingData();
    if (completed_)
    {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool should_send = !awaiting_ack_ ||
                             (now - last_send_time_) >= std::chrono::milliseconds(resend_period_ms_);
    if (should_send)
    {
      sendCurrentCommand();
    }
  }

  void openAndConfigureSerial()
  {
    serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
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
    tty.c_cc[VTIME] = timeoutToDeciseconds(timeout_sec_);

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      const std::string error = std::strerror(errno);
      closeSerial();
      throw std::runtime_error("tcsetattr failed: " + error);
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

  int timeoutToDeciseconds(double timeout_sec) const
  {
    if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0)
    {
      return 0;
    }

    const long deciseconds = static_cast<long>(std::ceil(timeout_sec * 10.0));
    return static_cast<int>(std::clamp(deciseconds, 1L, 255L));
  }

  void sendCurrentCommand()
  {
    const std::string packet = buildPacket(currentCommand());
    writeAll(packet);
    awaiting_ack_ = true;
    last_send_time_ = std::chrono::steady_clock::now();

    if (enable_serial_log_)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Sent STM32 command: %s",
          packet.substr(0, packet.size() - 2).c_str());
    }
  }

  void writeAll(const std::string &packet)
  {
    const char *data = packet.data();
    std::size_t remaining = packet.size();

    while (remaining > 0)
    {
      const ssize_t bytes_written = ::write(serial_fd_, data, remaining);
      if (bytes_written < 0)
      {
        if (errno == EINTR)
        {
          continue;
        }
        throw std::runtime_error("Serial write failed: " + std::string(std::strerror(errno)));
      }
      if (bytes_written == 0)
      {
        throw std::runtime_error("Serial write returned 0 bytes.");
      }

      data += bytes_written;
      remaining -= static_cast<std::size_t>(bytes_written);
    }

    if (tcdrain(serial_fd_) != 0)
    {
      throw std::runtime_error("tcdrain failed: " + std::string(std::strerror(errno)));
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
              this->get_logger(),
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

      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
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
        RCLCPP_INFO(this->get_logger(), "Received STM32 raw reply line: %s", line.c_str());
      }

      handleReplyLine(line);
      if (completed_)
      {
        return;
      }
    }
  }

  void handleReplyLine(const std::string &line)
  {
    const auto parsed_reply = parseReplyLine(line);
    if (!parsed_reply.has_value())
    {
      return;
    }

    const std::string expected_command_name = commandName(currentCommand().kind);
    if (parsed_reply->command_name != expected_command_name)
    {
      return;
    }

    if (parsed_reply->status == ReplyStatus::Ok)
    {
      if (enable_serial_log_)
      {
        RCLCPP_INFO(
            this->get_logger(),
            "STM32 acknowledged command '%s' with ok.",
            parsed_reply->command_name.c_str());
      }

      awaiting_ack_ = false;
      ++current_command_index_;
      if (current_command_index_ >= command_sequence_.size())
      {
        completed_ = true;
        RCLCPP_INFO(this->get_logger(), "STM32 behavior sequence completed.");
      }
      return;
    }

    if (enable_serial_log_)
    {
      RCLCPP_WARN(
          this->get_logger(),
          "STM32 replied 'eror' for '%s'; resending the same command.",
          parsed_reply->command_name.c_str());
    }
  }

  std::string port_;
  int baudrate_ = 115200;
  double timeout_sec_ = 0.05;
  int resend_period_ms_ = 200;
  int tick_period_ms_ = 20;
  bool enable_serial_log_ = true;
  bool enable_raw_reply_log_ = true;
  int serial_fd_ = -1;
  std::string incoming_buffer_;
  std::vector<Command> command_sequence_;
  std::size_t current_command_index_ = 0;
  bool awaiting_ack_ = false;
  bool completed_ = false;
  std::chrono::steady_clock::time_point last_send_time_{};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32BehaviorNode>());
  rclcpp::shutdown();
  return 0;
}
