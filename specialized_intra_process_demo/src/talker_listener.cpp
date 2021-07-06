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
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

using MessageT = std::vector<int>;
using MessageUniquePtr = std::unique_ptr<MessageT>;
int max_allocate_size = 200000;

class Talker : public rclcpp::Node
{
public:
  Talker(std::string node_name, std::string topic_name)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    count_(100)
  {
    pub_ = feature::create_intra_process_publisher<MessageT>(
      this, topic_name, 10);

    auto timer_callback = [&]() {
        count_ = count_ * 1.1;
        if (count_ > max_allocate_size) {
          throw std::bad_alloc();
        }
        auto msg = std::make_unique<MessageT>(count_);


        RCLCPP_INFO(get_logger(), "Publishing: size() = '%ld'", msg->size());
        pub_->publish(std::move(msg));
      };

    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  typename feature::Publisher<MessageT>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

class Listener : public rclcpp::Node
{
public:
  Listener(std::string node_name, std::string topic_name)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    auto sub_callback = [&](MessageUniquePtr msg) -> void {
        RCLCPP_INFO(get_logger(), "Subscribed: size() = '%ld'", msg->size());
      };

    sub_ = feature::create_intra_process_subscription<MessageT>(
      this, topic_name, 10, sub_callback);
  }

private:
  typename feature::Subscription<MessageT>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto topic_name = "/chatter";
  auto talker = std::make_shared<Talker>("talker_node", topic_name);
  auto listener = std::make_shared<Listener>("listener_node", topic_name);

  exec->add_node(talker);
  exec->add_node(listener);

  try {
    exec->spin();
  } catch (std::bad_alloc & e) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "catched %s", e.what());
  }

  return 0;
}
