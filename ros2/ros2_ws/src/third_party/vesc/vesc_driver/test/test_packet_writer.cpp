// Copyright 2026 Robocar contributors
// SPDX-License-Identifier: BSD-3-Clause

#include "vesc_driver/packet_writer.hpp"
#include "vesc_driver/vesc_packet.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
}  // namespace

int main()
{
  using vesc_driver::Buffer;
  using vesc_driver::VescPacketSetDuty;
  vesc_driver::PacketWriter writer;
  Buffer received;
  const Buffer expected = VescPacketSetDuty(0.045).frame();
  // A temporary real VESC packet remains valid until all short writes complete.
  writer.send(VescPacketSetDuty(0.045).frame(), [&received](const Buffer & bytes) {
    const auto size = std::min<std::size_t>(2, bytes.size());
    received.insert(received.end(), bytes.begin(), bytes.begin() + size);
    return size;
  });
  require(received == expected, "Partial writes truncated or corrupted the VESC frame");

  for (bool oversized : {false, true}) {
    bool rejected = false;
    try {
      writer.send(expected, [oversized](const Buffer & bytes) {
        return oversized ? bytes.size() + 1 : std::size_t{0};
      });
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    require(rejected, "Failed serial write was silently accepted");
  }
  bool propagated = false;
  try {
    writer.send(expected, [](const Buffer &) -> std::size_t {
      throw std::runtime_error("USB disconnected");
    });
  } catch (const std::runtime_error &) {
    propagated = true;
  }
  require(propagated, "USB error was swallowed");

  // Contending writers may split a frame, but must never interleave its bytes.
  received.clear();
  const Buffer forward = VescPacketSetDuty(0.05).frame();
  const Buffer reverse = VescPacketSetDuty(-0.05).frame();
  std::vector<std::thread> threads;
  for (int producer = 0; producer < 4; ++producer) {
    threads.emplace_back([&writer, &received, producer]() {
      for (int i = 0; i < 100; ++i) {
        writer.send(VescPacketSetDuty(producer % 2 ? -0.05 : 0.05).frame(),
          [&received](const Buffer & bytes) {
            received.push_back(bytes.front());
            std::this_thread::yield();
            return std::size_t{1};
          });
      }
    });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  require(received.size() == 400 * forward.size(), "Missing transmitted bytes");
  for (std::size_t i = 0; i < received.size(); i += forward.size()) {
    const auto start = received.begin() + i;
    require(std::equal(forward.begin(), forward.end(), start) ||
      std::equal(reverse.begin(), reverse.end(), start), "Concurrent frames interleaved");
  }
  std::cout << "USB: temporary packets, short writes, errors and 400 concurrent frames OK\n";
}
