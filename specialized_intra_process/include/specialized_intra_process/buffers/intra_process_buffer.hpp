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

  virtual void add_shared(MessageSharedPtr msg, uint64_t seq) = 0;
  virtual void add_unique(MessageUniquePtr msg, uint64_t seq) = 0;

  virtual bool consume_shared(MessageSharedPtr & msg, uint64_t seq) = 0;
  virtual bool consume_unique(MessageUniquePtr & msg, uint64_t seq) = 0;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename BufferT = std::unique_ptr<MessageT>>
class TypedIntraProcessBuffer : public IntraProcessBuffer<MessageT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TypedIntraProcessBuffer)

  using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageUniquePtr = std::unique_ptr<MessageT>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  explicit TypedIntraProcessBuffer(
    std::unique_ptr<BufferImplementationBase<BufferT>> buffer_impl,
    std::shared_ptr<Alloc> allocator = nullptr)
  {
    bool valid_type =
      (std::is_same<BufferT, MessageSharedPtr>::value ||
      std::is_same<BufferT, MessageUniquePtr>::value);
    if (!valid_type) {
      throw std::runtime_error("Creating TypedIntraProcessBuffer with not valid BufferT");
    }

    buffer_ = std::move(buffer_impl);

    if (!allocator) {
      message_allocator_ = std::make_shared<MessageAlloc>();
    } else {
      message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
    }
  }

  virtual ~TypedIntraProcessBuffer() {}

  void add_shared(MessageSharedPtr msg, uint64_t seq) override
  {
    add_shared_impl<BufferT>(std::move(msg), seq);
  }

  void add_unique(MessageUniquePtr msg, uint64_t seq) override
  {
    buffer_->enqueue(std::move(msg), seq);
  }

  bool consume_shared(MessageSharedPtr & msg, uint64_t seq) override
  {
    return consume_shared_impl<BufferT>(msg, seq);
  }

  bool consume_unique(MessageUniquePtr & msg, uint64_t seq) override
  {
    return consume_unique_impl<BufferT>(msg, seq);
  }

  void clear() override {buffer_->clear();}

  bool use_take_shared_method() const override
  {
    return std::is_same<BufferT, MessageSharedPtr>::value;
  }

private:
  std::unique_ptr<BufferImplementationBase<BufferT>> buffer_;

  std::shared_ptr<MessageAlloc> message_allocator_;

  // MessageSharedPtr to MessageSharedPtr
  template<typename DestinationT>
  typename std::enable_if<std::is_same<DestinationT, MessageSharedPtr>::value>::type
  add_shared_impl(MessageSharedPtr shared_msg, uint64_t seq)
  {
    buffer_->enqueue(std::move(shared_msg), seq);
  }

  // MessageSharedPtr to MessageUniquePtr
  template<typename DestinationT>
  typename std::enable_if<
    std::is_same<DestinationT, MessageUniquePtr>::value>::type
  add_shared_impl(MessageSharedPtr shared_msg, uint64_t seq)
  {
    // This should not happen: here a copy is unconditionally made, while the
    // intra-process manager can decide whether a copy is needed depending on
    // the number and the type of buffers

    // TODO(hsgwa): support deleter
    MessageUniquePtr unique_msg;
    // MessageDeleter *deleter =
    //     std::get_deleter<MessageDeleter, const MessageT>(shared_msg);
    auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
    MessageAllocTraits::construct(*message_allocator_.get(), ptr, *shared_msg);
    // if (deleter) {
    //   unique_msg = MessageUniquePtr(ptr, *deleter);
    // } else {
    unique_msg = MessageUniquePtr(ptr);
    // }

    buffer_->enqueue(std::move(unique_msg), seq);
  }

  // MessageSharedPtr to MessageSharedPtr
  template<typename OriginT>
  bool consume_shared_impl(
    typename std::enable_if<(std::is_same<OriginT, MessageSharedPtr>::value),
    MessageSharedPtr>::type & msg,
    uint64_t seq)
  {
    return buffer_->dequeue(msg, seq);
  }

  // MessageUniquePtr to MessageSharedPtr
  template<typename OriginT>
  bool consume_shared_impl(
    typename std::enable_if<(std::is_same<OriginT, MessageUniquePtr>::value),
    MessageSharedPtr>::type & msg,
    uint64_t seq)
  {
    MessageUniquePtr msg_;
    auto success = buffer_->dequeue(msg_, seq);
    if (success) {
      msg = std::move(msg_);
    }
    return success;
  }

  template<typename OriginT>
  bool consume_unique_impl(
    typename std::enable_if<(std::is_same<OriginT, MessageSharedPtr>::value),
    MessageUniquePtr>::type & msg,
    uint64_t seq)
  {
    MessageSharedPtr shared_msg;
    bool success = buffer_->dequeue(shared_msg, seq);

    if (success) {
      // MessageUniquePtr unique_msg;
      // // MessageDeleter *deleter =
      // //     std::get_deleter<MessageDeleter, const MessageT>(buffer_msg);
      auto ptr = MessageAllocTraits::allocate(*message_allocator_.get(), 1);
      MessageAllocTraits::construct(*message_allocator_.get(), ptr,
      *shared_msg);
      // // if (deleter) {
      // //   msg = MessageUniquePtr(ptr, *deleter);
      // // } else {
      msg = MessageUniquePtr(ptr);
      // // }
    }
    // TODO(hsgwa): implement deleter
    return success;
  }

  // MessageUniquePtr to MessageUniquePtr
  template<typename OriginT>
  bool
  consume_unique_impl(
    typename std::enable_if<(std::is_same<OriginT, MessageUniquePtr>::value),
    MessageUniquePtr>::type & msg,
    uint64_t seq)
  {
    return buffer_->dequeue(msg, seq);
  }
};

}  // namespace buffers
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__BUFFERS__INTRA_PROCESS_BUFFER_HPP_
