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
#include <cstdint>
#include <memory>
#include <utility>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "specialized_intra_process/buffers/intra_process_buffer.hpp"
#include "specialized_intra_process/buffers/ring_buffer_implementation.hpp"
#include "specialized_intra_process/specialized_intra_process.hpp"

using namespace std::chrono_literals;

class PublishNode : public rclcpp::Node
{
public:
  PublishNode()
  : Node("publish_node"), count_(0)
  {
    pub_ = feature::create_intra_process_publisher<int>(this, "notify", 10);

    auto publish_msg = [this]() -> void {
        auto message = std::make_unique<int>(count_++);

        std::cout << "Published: " << *message << std::endl;
        pub_->publish(std::move(message));
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }
  ~PublishNode() {}

private:
  int count_;
  feature::Publisher<int>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class SubscribeNode : public rclcpp::Node
{
public:
  SubscribeNode()
  : Node("subscribe_node")
  {
    sub_ = feature::create_intra_process_subscription<int>(
      this, "notify", 10,
      [&](std::unique_ptr<int> msg) -> void {std::cout << "Subscribed :" << *msg << std::endl;});
  }
  ~SubscribeNode() {}

private:
  feature::Subscription<int>::SharedPtr sub_;
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
