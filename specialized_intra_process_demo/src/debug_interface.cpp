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

#include <list>
#include <memory>
#include <string>
#include <utility>
#include <new>
#include <vector>

#include "notification_msgs/msg/notification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "specialized_intra_process/specialized_intra_process.hpp"
#include "std_msgs/msg/int32.hpp"

#define PING_TOPIC_NAME "ping"
#define PONG_TOPIC_NAME "pong"
using namespace std::chrono_literals;

using Message = int;
using MessageUniquePtr = std::unique_ptr<Message>;
using ROSMessage = std_msgs::msg::Int32;
using ROSMessageUniquePtr = ROSMessage::UniquePtr;

class Ping : public rclcpp::Node
{
public:
  explicit Ping(std::string node_name)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    count_(0)
  {
    auto sub_callback = [&](ROSMessageUniquePtr msg) -> void {
        RCLCPP_INFO(get_logger(), "Subscribed ros message: count = '%d'", *msg);
      };

    auto ping_debug_topic_name = PING_TOPIC_NAME + "/converted";
    pub_ = this->create_publisher<ROSMessage>(ping_debug_topic_name, 10);

    auto pong_debug_topic_name = PONG_TOPIC_NAME + "/converted";
    sub_ = this->create_subscription<ROSMessage>(pong_debug_topic_name, 10, sub_callback);

    auto timer_callback = [&]() {
        count_++;
        auto msg = std::make_unique<ROSMessage>();
        msg->data = count_;

        RCLCPP_INFO(get_logger(), "Publishing ros message: count = '%d'", *msg);
        pub_->publish(std::move(msg));
      };

    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::Publisher<ROSMessage>::SharedPtr pub_;
  rclcpp::Subscription<ROSMessage>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

class Pong : public rclcpp::Node
{
public:
  explicit Pong(std::string node_name)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    auto sub_callback = [&](MessageUniquePtr msg) -> void {
        RCLCPP_INFO(get_logger(), "Subscribed custom message : count = '%d'", *msg);
        RCLCPP_INFO(get_logger(), "Publishing custom message : count = '%d'", *msg);
        pub_->publish(std::move(msg));
      };

    pub_ = feature::create_intra_process_publisher<Message>(this, PONG_TOPIC_NAME, 10);
    auto to_ros_msg = [](const Message & source, ROSMessage & destination) {
        destination.data = source;
      };
    auto to_custom_msg = [](const ROSMessage & source, Message & destination) {
        destination = source.data;
      };

    pub_->set_conversion_to_ros_message(this, to_ros_msg);

    sub_ = feature::create_intra_process_subscription<Message>(
      this, PING_TOPIC_NAME, 10,
      sub_callback);
    sub_->set_conversion_to_custom_message(this, to_custom_msg);
  }

private:
  typename feature::Subscription<Message>::SharedPtr sub_;
  typename feature::Publisher<Message>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto ping = std::make_shared<Ping>("ping_node");
  auto pong = std::make_shared<Pong>("ping_node");

  exec->add_node(ping);
  exec->add_node(pong);

  try {
    exec->spin();
  } catch (std::bad_alloc & e) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "catched %s", e.what());
  }

  return 0;
}
