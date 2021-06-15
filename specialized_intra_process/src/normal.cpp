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

#include <chrono>
#include <memory>
#include <utility>

#include "notification_msgs/msg/notification.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
class PublishNode : public rclcpp::Node
{
public:
  PublishNode()
  : Node("publish_node", rclcpp::NodeOptions().use_intra_process_comms(true)), count_(0)
  {
    pub_ = create_publisher<notification_msgs::msg::Notification>("notify", 10);

    auto publish_msg = [&]() -> void {
        auto message = std::make_unique<notification_msgs::msg::Notification>();
        pub_->publish(std::move(message));
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  int count_;
  rclcpp::Publisher<notification_msgs::msg::Notification>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class SubscribeNode : public rclcpp::Node
{
public:
  SubscribeNode()
  : Node("subscribe_node", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    sub_ = create_subscription<notification_msgs::msg::Notification>(
      "notify", 10, [&](notification_msgs::msg::Notification::UniquePtr msg) -> void {
        (void)msg;
        std::cout << " recv" << std::endl;
      });
  }

private:
  rclcpp::Subscription<notification_msgs::msg::Notification>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto pub_node = std::make_shared<PublishNode>();
  exec->add_node(pub_node);

  auto sub_node = std::make_shared<SubscribeNode>();
  exec->add_node(sub_node);

  exec->spin();
  rclcpp::shutdown();

  return 0;
}
