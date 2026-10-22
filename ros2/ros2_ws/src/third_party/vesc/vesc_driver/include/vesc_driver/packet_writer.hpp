// [robocar] Keep each serial frame alive and serialized through its complete write.
#ifndef VESC_DRIVER__PACKET_WRITER_HPP_
#define VESC_DRIVER__PACKET_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace vesc_driver
{

class PacketWriter
{
public:
  // write() must be synchronous. SerialPort::send() may write only part of a frame.
  template<typename Write>
  void send(const std::vector<uint8_t> & frame, Write write)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t offset = 0;
    while (offset < frame.size()) {
      const std::vector<uint8_t> remaining(frame.begin() + offset, frame.end());
      const auto written = write(remaining);
      if (written == 0 || written > remaining.size()) {
        throw std::runtime_error("Incomplete VESC serial write");
      }
      offset += written;
    }
  }

private:
  std::mutex mutex_;
};

}  // namespace vesc_driver
#endif  // VESC_DRIVER__PACKET_WRITER_HPP_
