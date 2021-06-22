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

#pragma once

#ifndef SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_PUBLISHER_HPP_
#define SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_PUBLISHER_HPP_

#include <string>
#include <memory>

#include "publisher_wrapper.hpp"

namespace feature
{
template<
  typename MessageT,
  typename AllocatorT = std::allocator<void>,
  typename PublisherT = feature::Publisher<MessageT, AllocatorT>
>
typename PublisherT::SharedPtr
create_intra_process_publisher(
  rclcpp::Node * node,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
  rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
)
{
  auto pub = node->create_publisher<notification_msgs::msg::Notification>(
    topic_name, qos, options);

  auto pub_wrapper = std::make_shared<PublisherT>(node, pub, options);

  // This is used for setting up things like intra process comms which
  // require this->shared_from_this() which cannot be called from
  // the constructor.
  pub_wrapper->post_init_setup(node, pub);

  return pub_wrapper;
}
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_PUBLISHER_HPP_
