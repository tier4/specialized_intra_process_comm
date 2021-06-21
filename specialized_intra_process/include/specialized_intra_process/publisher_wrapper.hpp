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

#ifndef SPECIALIZED_INTRA_PROCESS__PUBLISHER_WRAPPER_HPP_
#define SPECIALIZED_INTRA_PROCESS__PUBLISHER_WRAPPER_HPP_

#include <memory>
#include <utility>

#include "intra_process_manager.hpp"
#include "notification_msgs/msg/notification.hpp"
#include "publisher_wrapper_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace feature
{
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>
>
class Publisher : public feature::PublisherBase
{
public:
  using NotificationMessageT = notification_msgs::msg::Notification;
  using RosPublisherT = rclcpp::Publisher<NotificationMessageT, AllocatorT>;

  using MessageAllocatorTraits =
    rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT, AllocatorT>)

  Publisher(
    rclcpp::Node * node, std::shared_ptr<RosPublisherT> pub,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  : PublisherBase(pub), message_allocator_(new MessageAllocator(
        *options.get_allocator().get())),
    pub_(pub), node_(node)
  {
  }

  ~Publisher() {}

  void post_init_setup(rclcpp::Node * node, std::shared_ptr<RosPublisherT> pub)
  {
    (void) pub;
    auto node_base = node->get_node_base_interface();
    auto context = node_base->get_context();
    auto ipm = context->get_sub_context<IntraProcessManager>();

    uint64_t intra_process_publisher_id = ipm->add_publisher(this->shared_from_this());
    this->setup_intra_process(intra_process_publisher_id, ipm);
  }

  void publish(std::unique_ptr<MessageT> msg)
  {
    auto seq = do_intra_process_publish(std::move(msg));
    auto notify_msg = std::make_unique<notification_msgs::msg::Notification>();
    notify_msg->seq = seq;
    notify_msg->header.stamp = node_->now();
    pub_->publish(std::move(notify_msg));
  }

  uint64_t do_intra_process_publish(std::unique_ptr<MessageT> msg)
  {
    auto ipm = weak_ipm_.lock();

    auto seq = ipm->template do_intra_process_publish<MessageT, AllocatorT>(
      intra_process_publisher_id_, std::move(msg), message_allocator_);
    return seq;
  }
  std::shared_ptr<MessageAllocator> message_allocator_;
  std::shared_ptr<RosPublisherT> pub_;
  rclcpp::Node * node_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__PUBLISHER_WRAPPER_HPP_
