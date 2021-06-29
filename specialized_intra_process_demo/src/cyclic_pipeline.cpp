// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "specialized_intra_process/specialized_intra_process.hpp"

using namespace std::chrono_literals;
void to_ros_msg(const int &source, std_msgs::msg::Int32 &destination) {
  destination.data = source;
}

void to_custom_msg(const std_msgs::msg::Int32 &source, int &destination) {
  destination = source.data;
}

// This node receives an Int32, waits 1 second, then increments and sends it.
struct IncrementerPipe : public rclcpp::Node
{
  IncrementerPipe(const std::string & name, const std::string & in, const std::string & out)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    // Create a publisher on the output topic.

    pub = feature::create_intra_process_publisher<int>(this, out, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;
    // Create a subscription on the input topic.
    sub = feature::create_intra_process_subscription<int>(
      this, in, 10, [captured_pub](std::unique_ptr<int> msg) {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }
        printf(
          "Received message with value:         %d, and address: 0x%" PRIXPTR "\n", *msg,
          reinterpret_cast<std::uintptr_t>(msg.get()));
        printf("  sleeping for 1 second...\n");
        if (!rclcpp::sleep_for(1s)) {
          return;  // Return if the sleep failed (e.g. on ctrl-c).
        }
        printf("  done.\n");
        (*msg)++;  // Increment the message's data.
        printf(
          "Incrementing and sending with value: %d, and address: 0x%" PRIXPTR "\n", *msg,
          reinterpret_cast<std::uintptr_t>(msg.get()));
        pub_ptr->publish(std::move(msg));  // Send the message along to the output topic.
      });

    pub->set_conversion_to_ros_message(this, to_ros_msg, "/converted/pub");
    sub->set_conversion_to_custom_message(this, to_custom_msg, "/converted/sub");
  }

  feature::Subscription<int>::SharedPtr sub;
  feature::Publisher<int>::SharedPtr pub;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create a simple loop by connecting the in and out topics of two IncrementerPipe's.
  // The expectation is that the address of the message being passed between them never changes.
  auto pipe1 = std::make_shared<IncrementerPipe>("pipe1", "topic1", "topic2");
  auto pipe2 = std::make_shared<IncrementerPipe>("pipe2", "topic2", "topic1");
  rclcpp::sleep_for(1s);  // Wait for subscriptions to be established to avoid race conditions.
  // Publish the first message (kicking off the cycle).
  std::unique_ptr<int> msg(new int());
  *msg = 42;
  printf(
    "Published first message with value:  %d, and address: 0x%" PRIXPTR "\n", *msg,
    reinterpret_cast<std::uintptr_t>(msg.get()));
  pipe1->pub->publish(std::move(msg));

  executor.add_node(pipe1);
  executor.add_node(pipe2);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
