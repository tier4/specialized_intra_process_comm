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

#ifndef SPECIALIZED_INTRA_PROCESS__ROS_MESSAGE_PUBLISHER_HPP_
#define SPECIALIZED_INTRA_PROCESS__ROS_MESSAGE_PUBLISHER_HPP_

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"


namespace feature
{
template<typename MessageT>
class RosMessagePublisherBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RosMessagePublisherBase)

  RosMessagePublisherBase() {}

  virtual ~RosMessagePublisherBase() {}

  virtual void publish_ros_message(const MessageT & custom_message) = 0;
};

template<
  typename MessageT,
  typename RosMessageT,
  typename ConversionT
>
class RosMessagePublisher : public RosMessagePublisherBase<MessageT>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RosMessagePublisher)

  template<typename PublisherT>
  RosMessagePublisher(std::shared_ptr<PublisherT> publisher, ConversionT conversion)
  : pub_(publisher), conversion_(conversion)
  {
  }

  void publish_ros_message(const MessageT & custom_message) override
  {
    if (pub_->get_subscription_count() == 0) {
      return;
    }
    RosMessageT ros_message;
    conversion_(custom_message, ros_message);
    pub_->publish(std::move(ros_message));
  }

public:
  ConversionT conversion_;
  typename rclcpp::Publisher<RosMessageT>::SharedPtr pub_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__ROS_MESSAGE_PUBLISHER_HPP_
