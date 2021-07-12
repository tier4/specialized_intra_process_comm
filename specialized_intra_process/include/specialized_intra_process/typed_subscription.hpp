// Copyright 2021 Research Institute of Systems Planning, Inc.
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

#ifndef SPECIALIZED_INTRA_PROCESS__TYPED_SUBSCRIPTION_HPP_
#define SPECIALIZED_INTRA_PROCESS__TYPED_SUBSCRIPTION_HPP_

#include <memory>
#include <utility>

#include "create_intra_process_buffer.hpp"
#include "intra_process_buffer_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "subscription_base.hpp"

namespace feature
{
template<
  typename CallbackMessageT,
  typename AllocatorT = std::allocator<void>,
  typename Deleter = std::default_delete<CallbackMessageT>
>
class TypedSubscription : public SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TypedSubscription)

  using MessageUniquePtr = std::unique_ptr<CallbackMessageT, Deleter>;
  using MessageSharedPtr = std::shared_ptr<const CallbackMessageT>;

  using BufferSharedPtr =
    typename feature::buffers::IntraProcessBuffer<
    CallbackMessageT, AllocatorT, Deleter>::SharedPtr;

  TypedSubscription()
  : SubscriptionBase() {}

  ~TypedSubscription() {}
  void post_init_setup(
    rclcpp::SubscriptionBase::SharedPtr sub,
    bool use_take_shared_method)
  {
    SubscriptionBase::post_init_setup(sub);

    IntraProcessBufferType buffer_type;
    if (use_take_shared_method) {
      buffer_type = IntraProcessBufferType::SharedPtr;
    } else {
      buffer_type = IntraProcessBufferType::UniquePtr;
    }
    rmw_qos_profile_t qos_profile = get_actual_qos().get_rmw_qos_profile();
    buffer_ = feature::create_intra_process_buffer<CallbackMessageT, AllocatorT, Deleter>(
      buffer_type, qos_profile);
  }

  bool use_take_shared_method() const
  {
    return buffer_->use_take_shared_method();
  }

  bool consume_unique(MessageUniquePtr & msg, uint64_t seq)
  {
    return buffer_->consume_unique(msg, seq);
  }

  bool consume_shared(MessageSharedPtr & msg, uint64_t seq)
  {
    return buffer_->consume_shared(msg, seq);
  }

  void provide_intra_process_message(MessageSharedPtr message, uint64_t seq)
  {
    buffer_->add_shared(std::move(message), seq);
  }

  void provide_intra_process_message(MessageUniquePtr message, uint64_t seq)
  {
    buffer_->add_unique(std::move(message), seq);
  }
  BufferSharedPtr buffer_;
};

}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__TYPED_SUBSCRIPTION_HPP_
