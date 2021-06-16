// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SPECIALIZED_INTRA_PROCESS__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
#define SPECIALIZED_INTRA_PROCESS__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "buffer_implementation_base.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/visibility_control.hpp"

namespace feature
{
namespace buffers
{
template<typename BufferT>
struct BufferContainer
{
  BufferT data;
  uint64_t seq;
};

template<typename BufferT>
class RingBufferImplementation : public BufferImplementationBase<BufferT>
{
public:
  explicit RingBufferImplementation(size_t capacity)
  : capacity_(capacity),
    ring_buffer_(capacity),
    write_index_(capacity_ - 1),
    read_index_(0),
    size_(0)
  {
    if (capacity == 0) {
      throw std::invalid_argument("capacity must be a positive, non-zero value");
    }
  }

  virtual ~RingBufferImplementation() {}

  void enqueue(BufferT request, uint64_t seq)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    write_index_ = next(write_index_);
    ring_buffer_[write_index_].data = std::move(request);
    ring_buffer_[write_index_].seq = seq;

    if (is_full()) {
      read_index_ = next(read_index_);
    } else {
      size_++;
    }
  }

  BufferT dequeue(uint64_t seq)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_data()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Calling dequeue on empty intra-process buffer");
      throw std::runtime_error("Calling dequeue on empty intra-process buffer");
    }

    // find and return seq-mached-message
    auto read_index = read_index_;
    for (size_t i = 0; i < capacity_; i++) {
      if (ring_buffer_[read_index_].seq == seq) {
        auto request = std::move(ring_buffer_[read_index].data);
        read_index_ = next(read_index);
        size_--;
        return request;
      }
      read_index = next(read_index);
    }

    // TODO(hsgwa): implement error pattern
    auto request = std::move(ring_buffer_[read_index_].data);
    read_index_ = next(read_index_);

    return request;
  }

  inline size_t next(size_t val) {return (val + 1) % capacity_;}

  inline bool has_data() const {return size_ != 0;}

  inline bool is_full() {return size_ == capacity_;}

  void clear() {}

private:
  size_t capacity_;

  using BufferContainerT = BufferContainer<BufferT>;
  std::vector<BufferContainerT> ring_buffer_;

  size_t write_index_;
  size_t read_index_;
  size_t size_;

  std::mutex mutex_;
};

}  // namespace buffers
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
