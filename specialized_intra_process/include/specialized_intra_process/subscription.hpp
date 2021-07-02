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

#ifndef SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_HPP_
#define SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_HPP_

#include <memory>
#include <functional>
#include <utility>
#include <string>

#include "create_intra_process_buffer.hpp"
#include "intra_process_buffer_type.hpp"
#include "intra_process_manager.hpp"
#include "notification_msgs/msg/notification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "typed_subscription.hpp"

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

  using TypedSubscriptionT =
    TypedSubscription<CallbackMessageT, MessageAllocatorT, MessageDeleter>;

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

  template<typename FunctionT>
  using function_traits = rclcpp::function_traits::function_traits<FunctionT>;

  Subscription()
  : sub_(nullptr), notify_sub_(nullptr)
  {
  }

  ~Subscription()
  {
  }

  template<typename CallbackT>
  void post_init_setup(
    rclcpp::Node * node,
    typename NotifySubscriptionT::SharedPtr notify_sub,
    bool use_take_shared_method,
    CallbackT callback)
  {
    sub_ = std::make_shared<TypedSubscriptionT>();
    sub_->post_init_setup(notify_sub, use_take_shared_method);
    notify_sub_ = notify_sub;
    callback_ = callback;

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


  template
  <
    typename ConversionT,
    typename RosMessageT =
    typename std::remove_const<
      typename std::remove_reference<
        typename function_traits<ConversionT>::template argument_type<0>>
      ::type>::type
  >
  void set_conversion_to_custom_message(
    rclcpp::Node * node, ConversionT conversion, std::string suffix = "/converted")
  {
    auto callback_wrapper = [&, conversion](typename RosMessageT::UniquePtr msg) {
        auto custom_msg = std::make_unique<CallbackMessageT>();
        conversion(*msg, *custom_msg);
        callback_(std::move(custom_msg));
      };

    auto topic_name = sub_->get_topic_name() + suffix;
    auto qos = sub_->get_actual_qos();
    ros_msg_sub_ = node->create_subscription<RosMessageT>(topic_name, qos, callback_wrapper);
  }

private:
  std::shared_ptr<TypedSubscriptionT> sub_;
  typename NotifySubscriptionT::SharedPtr notify_sub_;
  std::function<void(MessageUniquePtr)> callback_;
  rclcpp::SubscriptionBase::SharedPtr ros_msg_sub_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_HPP_
