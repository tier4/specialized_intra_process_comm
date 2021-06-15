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

#ifndef SPECIALIZED_INTRA_PROCESS__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
#define SPECIALIZED_INTRA_PROCESS__BUFFERS__INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <type_traits>
#include <utility>

#include "buffer_implementation_base.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/allocator/allocator_deleter.hpp"

namespace feature
{
namespace buffers
{
class IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBufferBase)

  virtual ~IntraProcessBufferBase() {}

  virtual void clear() = 0;

  virtual bool has_data() const = 0;
  virtual bool use_take_shared_method() const = 0;
};

template<typename MessageT>
class IntraProcessBuffer : public IntraProcessBufferBase
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(IntraProcessBuffer)

  virtual ~IntraProcessBuffer() {}

  using MessageUniquePtr = std::unique_ptr<MessageT>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  // virtual void add_shared(MessageSharedPtr msg) = 0;
  virtual uint64_t add_unique(MessageUniquePtr msg) = 0;

  // virtual MessageSharedPtr consume_shared() = 0;
  virtual MessageUniquePtr consume_unique(uint64_t seq) = 0;
};

template<typename MessageT, typename BufferT = std::unique_ptr<MessageT>>
class TypedIntraProcessBuffer : public IntraProcessBuffer<MessageT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TypedIntraProcessBuffer)

  using MessageUniquePtr = std::unique_ptr<MessageT>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  explicit TypedIntraProcessBuffer(std::unique_ptr<BufferImplementationBase<BufferT>> buffer_impl)
  {
    bool valid_type =
      (std::is_same<BufferT, MessageSharedPtr>::value ||
      std::is_same<BufferT, MessageUniquePtr>::value);
    if (!valid_type) {
      throw std::runtime_error("Creating TypedIntraProcessBuffer with not valid BufferT");
    }

    buffer_ = std::move(buffer_impl);
  }

  virtual ~TypedIntraProcessBuffer() {}

  // void add_shared(MessageSharedPtr msg) override
  // {
  //   add_shared_impl<BufferT>(std::move(msg));
  // }

  uint64_t add_unique(MessageUniquePtr msg) override
  {
    auto seq = buffer_->enqueue(std::move(msg));
    return seq;
  }

  // MessageSharedPtr consume_shared() override
  // {
  //   return consume_shared_impl<BufferT>();
  // }

  MessageUniquePtr consume_unique(uint64_t seq) override
  {
    return consume_unique_impl<BufferT>(seq);
  }

  bool has_data() const override {return buffer_->has_data();}

  void clear() override {buffer_->clear();}

  bool use_take_shared_method() const override
  {
    return std::is_same<BufferT, MessageSharedPtr>::value;
  }

private:
  std::unique_ptr<BufferImplementationBase<BufferT>> buffer_;

  // MessageSharedPtr to MessageSharedPtr
  template<typename DestinationT>
  typename std::enable_if<std::is_same<DestinationT, MessageSharedPtr>::value>::type
  add_shared_impl(MessageSharedPtr shared_msg)
  {
    buffer_->enqueue(std::move(shared_msg));
  }

  // MessageSharedPtr to MessageSharedPtr
  template<typename OriginT>
  typename std::enable_if<std::is_same<OriginT, MessageSharedPtr>::value, MessageSharedPtr>::type
  consume_shared_impl()
  {
    return buffer_->dequeue();
  }

  // MessageUniquePtr to MessageUniquePtr
  template<typename OriginT>
  typename std::enable_if<(std::is_same<OriginT, MessageUniquePtr>::value), MessageUniquePtr>::type
  consume_unique_impl(uint64_t seq)
  {
    return buffer_->dequeue(seq);
  }
};

}  // namespace buffers
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
