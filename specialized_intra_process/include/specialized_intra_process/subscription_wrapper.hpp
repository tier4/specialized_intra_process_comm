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

#ifndef SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_WRAPPER_HPP_
#define SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_WRAPPER_HPP_

#include <memory>

#include "create_intra_process_buffer.hpp"
#include "intra_process_buffer_type.hpp"
#include "intra_process_manager.hpp"
#include "notification_msgs/msg/notification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "typed_subscription_wrapper_base.hpp"

namespace feature
{
template<
  typename CallbackMessageT,
  typename AllocatorT = std::allocator<void>
>
class Subscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  using MessageAllocTraits =
    rclcpp::allocator::AllocRebind<CallbackMessageT, AllocatorT>;
  using MessageAllocatorT = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocatorT, CallbackMessageT>;
  using MessageUniquePtr = std::unique_ptr<CallbackMessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const CallbackMessageT>;

  using TypedSubscriptionBaseT =
    TypedSubscriptionBase<CallbackMessageT, MessageAllocatorT, MessageDeleter>;

  using NotificationT = notification_msgs::msg::Notification;
  using NotificationAllocTraits =
    rclcpp::allocator::AllocRebind<NotificationT, AllocatorT>;
  using NotificationAllocatorT =
    typename NotificationAllocTraits::allocator_type;
  using NotificationDeleter =
    rclcpp::allocator::Deleter<NotificationAllocatorT, NotificationT>;
  using NotificationUniquePtr =
    std::unique_ptr<NotificationT, NotificationDeleter>;

  using NotificationMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<NotificationT,
      AllocatorT>;
  using NotifySubscriptionT =
    typename rclcpp::Subscription<NotificationT, AllocatorT,
      NotificationMemoryStrategyT>;

  Subscription()
  {
  }

  ~Subscription()
  {
  }

  void post_init_setup(
    rclcpp::Node * node, typename NotifySubscriptionT::SharedPtr notify_sub,
    bool use_take_shared_method)
  {
    sub_ = std::make_shared<TypedSubscriptionBaseT>();
    sub_->post_init_setup(notify_sub, use_take_shared_method);
    notify_sub_ = notify_sub;

    auto node_base = node->get_node_base_interface();
    auto context = node_base->get_context();
    auto ipm = context->get_sub_context<feature::IntraProcessManager>();

    uint64_t intra_process_subscription_id = ipm->add_subscription(sub_);
    sub_->setup_intra_process(intra_process_subscription_id, ipm);
  }

  bool consume(MessageUniquePtr & msg, uint64_t seq)
  {
    return sub_->consume_unique(msg, seq);
  }

  bool consume(MessageSharedPtr & msg, uint64_t seq)
  {
    return sub_->consume_shared(msg, seq);
  }

  // private:
  std::shared_ptr<TypedSubscriptionBaseT> sub_;
  typename NotifySubscriptionT::SharedPtr notify_sub_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_WRAPPER_HPP_
