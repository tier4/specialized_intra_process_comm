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

#ifndef SPECIALIZED_INTRA_PROCESS__PUBLISHER_HPP_
#define SPECIALIZED_INTRA_PROCESS__PUBLISHER_HPP_

#include <memory>
#include <utility>
#include <string>
#include <type_traits>

#include "intra_process_manager.hpp"
#include "notification_msgs/msg/notification.hpp"
#include "publisher_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros_message_publisher.hpp"

namespace feature
{
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>
>
class Publisher : public feature::PublisherBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, AllocatorT>)

  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocator, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<const MessageT>;

  using NotificationT = notification_msgs::msg::Notification;
  using NotificationAllocatorTraits =
    rclcpp::allocator::AllocRebind<NotificationT, AllocatorT>;
  using NotificationAllocator = typename NotificationAllocatorTraits::allocator_type;
  using NotificationDeleter = rclcpp::allocator::Deleter<NotificationAllocator, NotificationT>;
  using NotificationUniquePtr = std::unique_ptr<NotificationT, NotificationDeleter>;

  using NotificationPublisherT = rclcpp::Publisher<NotificationT, AllocatorT>;

  template<typename FunctionT>
  using function_traits = rclcpp::function_traits::function_traits<FunctionT>;


  Publisher(
    rclcpp::Node * node, std::shared_ptr<NotificationPublisherT> pub,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  : PublisherBase(pub),
    message_allocator_(new MessageAllocator(*options.get_allocator().get())),
    notification_allocator_(new NotificationAllocator(*options.get_allocator().get())),
    pub_(pub),
    clock_(node->get_clock()),
    ros_msg_pub_(nullptr)
  {
    using rclcpp::allocator::set_allocator_for_deleter;
    set_allocator_for_deleter(&message_deleter_, message_allocator_.get());
    set_allocator_for_deleter(&notification_deleter_, notification_allocator_.get());
  }

  ~Publisher() {}

  void post_init_setup(rclcpp::Node * node, std::shared_ptr<NotificationPublisherT> pub)
  {
    (void) pub;
    auto node_base = node->get_node_base_interface();
    auto context = node_base->get_context();
    auto ipm = context->get_sub_context<IntraProcessManager>();

    uint64_t intra_process_publisher_id = ipm->add_publisher(this->shared_from_this());
    this->setup_intra_process(intra_process_publisher_id, ipm);
  }

  void publish(MessageUniquePtr msg)
  {
    if (ros_msg_pub_) {
      ros_msg_pub_->publish_ros_message(*msg);
    }

    auto seq = do_intra_process_publish(std::move(msg));
    auto ptr = NotificationAllocatorTraits::allocate(*notification_allocator_.get(), 1);
    NotificationAllocatorTraits::construct(*notification_allocator_.get(), ptr);
    NotificationUniquePtr notify_msg(ptr, notification_deleter_);
    notify_msg->seq = seq;
    if (auto clock = clock_.lock()) {
      notify_msg->header.stamp = clock->now();
    }
    pub_->publish(std::move(notify_msg));
  }

  uint64_t do_intra_process_publish(MessageUniquePtr msg)
  {
    auto ipm = weak_ipm_.lock();

    auto seq = ipm->template do_intra_process_publish<MessageT, MessageAllocator, MessageDeleter>(
      intra_process_publisher_id_, std::move(msg), message_allocator_);
    return seq;
  }

  template<
    typename ConversionT,
    typename RosMessageT =
    typename std::remove_const<
      typename std::remove_reference<
        typename function_traits<ConversionT>::template argument_type<1>
      >::type
    >::type
  >
  void set_conversion_to_ros_message(
    rclcpp::Node * node, ConversionT conversion,
    std::string suffix = "/converted")
  {
    auto topic_name = pub_->get_topic_name() + suffix;
    auto qos = pub_->get_actual_qos();
    // TODO(hsgwa): support allocator;
    auto pub = node->create_publisher<RosMessageT>(topic_name, qos);
    using RosMessagePublisherT = RosMessagePublisher<MessageT, RosMessageT, ConversionT>;
    auto rospub = std::make_shared<RosMessagePublisherT>(pub, conversion);
    ros_msg_pub_ = rospub;
  }

  std::shared_ptr<MessageAllocator> message_allocator_;
  MessageDeleter message_deleter_;
  std::shared_ptr<NotificationAllocator> notification_allocator_;
  NotificationDeleter notification_deleter_;
  std::shared_ptr<NotificationPublisherT> pub_;
  std::weak_ptr<rclcpp::Clock> clock_;
  typename RosMessagePublisherBase<MessageT>::SharedPtr ros_msg_pub_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__PUBLISHER_HPP_
