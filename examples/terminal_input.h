#pragma once

#include <cstring>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace rs01_examples {

// -----------------------------------------------------------------------------
// 终端键盘输入工具
// -----------------------------------------------------------------------------

/**
 * @brief 将终端临时切换为单键非阻塞输入模式。
 *
 * 构造时关闭规范模式和回显，并把 stdin 设置为非阻塞；析构时恢复原始设置。
 */
class TerminalInput {
public:
  // ---------------------------------------------------------------------------
  // 生命周期
  // ---------------------------------------------------------------------------

  /**
   * @brief 保存当前终端状态，并切换到非阻塞单键读取模式。
   *
   * @throws std::runtime_error 无法读取或设置终端属性时抛出。
   */
  TerminalInput() {
    if (!isatty(STDIN_FILENO)) {
      return;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      throw std::runtime_error(std::string("tcgetattr failed: ") +
                               std::strerror(errno));
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      throw std::runtime_error(std::string("tcsetattr failed: ") +
                               std::strerror(errno));
    }
    termios_active_ = true;

    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ < 0) {
      restore();
      throw std::runtime_error(std::string("fcntl F_GETFL failed: ") +
                               std::strerror(errno));
    }
    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
      restore();
      throw std::runtime_error(std::string("fcntl F_SETFL failed: ") +
                               std::strerror(errno));
    }
    flags_active_ = true;
  }

  /**
   * @brief 恢复构造前的终端模式和文件描述符标志。
   */
  ~TerminalInput() { restore(); }

  TerminalInput(const TerminalInput &) = delete;
  TerminalInput &operator=(const TerminalInput &) = delete;

  // ---------------------------------------------------------------------------
  // 按键读取
  // ---------------------------------------------------------------------------

  /**
   * @brief 尝试读取一个按键。
   *
   * @param key 成功读取时写入按键字符。
   * @return true 表示读到一个按键；false 表示当前没有输入。
   */
  bool read_key(char &key) const {
    char buffer = 0;
    const ssize_t n = read(STDIN_FILENO, &buffer, 1);
    if (n == 1) {
      key = buffer;
      return true;
    }
    return false;
  }

private:
  // ---------------------------------------------------------------------------
  // 状态恢复
  // ---------------------------------------------------------------------------

  /**
   * @brief 恢复终端原始配置。该函数不抛异常，便于在析构中调用。
   */
  void restore() {
    if (flags_active_) {
      fcntl(STDIN_FILENO, F_SETFL, original_flags_);
      flags_active_ = false;
    }
    if (termios_active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      termios_active_ = false;
    }
  }

  termios original_termios_ {};
  int original_flags_ = -1;
  bool termios_active_ = false;
  bool flags_active_ = false;
};

} // namespace rs01_examples
