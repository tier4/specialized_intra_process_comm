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

#ifndef SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_SUBSCRIPTION_HPP_
#define SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_SUBSCRIPTION_HPP_

#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "subscription.hpp"

namespace feature
{
template<
  typename MessageT,
  typename CallbackT,
  typename AllocatorT = std::allocator<void>,
  typename CallbackMessageT =
  typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
  typename SubscriptionT = feature::Subscription<MessageT, AllocatorT>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::
  MessageMemoryStrategy<CallbackMessageT, AllocatorT>>
typename std::shared_ptr<SubscriptionT> create_intra_process_subscription(
  rclcpp::Node * node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  CallbackT && callback,
  const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
  (rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()),
  typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
  (MessageMemoryStrategyT::create_default()))
{
  using NotificationT = notification_msgs::msg::Notification;
  using NotificationAllocTraits = rclcpp::allocator::AllocRebind<NotificationT, AllocatorT>;
  using NotificationAllocatorT = typename NotificationAllocTraits::allocator_type;
  using NotificationDeleter =
    rclcpp::allocator::Deleter<NotificationAllocatorT, NotificationT>;
  using NotificationUniquePtr = std::unique_ptr<NotificationT, NotificationDeleter>;
  using NotificationMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<NotificationT, AllocatorT>;

  auto sub_wrapper = std::make_shared<SubscriptionT>();
  std::weak_ptr<SubscriptionT> sub_wrapper_weak = sub_wrapper;

  auto callback_wrapper =
    [callback, sub_wrapper_weak](NotificationUniquePtr msg) -> void {
      auto sub_wrapper = sub_wrapper_weak.lock();
      if (!sub_wrapper) {
        return;
      }
      using rclcpp::function_traits::function_traits;
      using CallbackArgT = typename function_traits<CallbackT>::template argument_type<0>;
      CallbackArgT buf_msg;

      auto success = sub_wrapper->consume(buf_msg, msg->seq);
      if (success) {
        callback(std::move(buf_msg));
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger("specialized_intra_process"),
          "Failed to get the message. Executing callback was cancelled.");
      }
    };

  auto notification_mem_strat =
    std::make_shared<NotificationMemoryStrategyT>(options.allocator);
  auto sub = node->create_subscription<NotificationT>(
    topic_name, qos, callback_wrapper, options, notification_mem_strat);


  // define any_calback to call "use_take_shared_method"
  rclcpp::AnySubscriptionCallback<CallbackMessageT, AllocatorT>
  any_callback(options.get_allocator());
  any_callback.set(callback);
  auto use_take_shared_method = any_callback.use_take_shared_method();

  // This is used for setting up things like intra process comms which
  // require this->shared_from_this() which cannot be called from
  // the constructor.
  // And because of circular reference.
  sub_wrapper->post_init_setup(node, sub, use_take_shared_method, callback);

  return sub_wrapper;
}
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_SUBSCRIPTION_HPP_
