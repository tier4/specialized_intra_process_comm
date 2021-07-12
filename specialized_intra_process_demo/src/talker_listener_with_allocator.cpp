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
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "specialized_intra_process_comm/specialized_intra_process_comm.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tlsf_cpp/tlsf.hpp"

using namespace std::chrono_literals;
template<typename T>
using TLSFAllocator = tlsf_heap_allocator<T>;

using Alloc = TLSFAllocator<void>;
// using Alloc = std::allocator<void>;

using MessageT = std::vector<int, TLSFAllocator<int>>;
using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
using MessageAlloc = MessageAllocTraits::allocator_type;
using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;
using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
using MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT,
    Alloc>;

template<typename Alloc = std::allocator<void>>
class Talker : public rclcpp::Node
{
public:
  Talker(std::string node_name, std::string topic_name, std::shared_ptr<Alloc> alloc)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)),
    count_(100)
  {
    rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
    publisher_options.allocator = alloc;

    pub_ = feature::create_intra_process_publisher<MessageT>(
      this, topic_name, 10, publisher_options);

    message_alloc = *alloc;
    rclcpp::allocator::set_allocator_for_deleter(&message_deleter, &message_alloc);

    auto timer_callback = [&]() {
        count_ = count_ * 1.1;

        auto msg_ptr = MessageAllocTraits::allocate(message_alloc, 1);

        // use pass allocator to std::vector constructor
        // constructor definition:
        //  explicit std::vector(size_type n, const Allocator& a = Allocator());
        MessageAllocTraits::construct(message_alloc, msg_ptr, count_, message_alloc);

        MessageUniquePtr msg(msg_ptr, message_deleter);

        RCLCPP_INFO(get_logger(), "Publishing: size() = '%ld'", msg->size());
        pub_->publish(std::move(msg));
      };

    timer_ = this->create_wall_timer(100ms, timer_callback);
  }

private:
  typename feature::Publisher<MessageT, Alloc>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
  int count_max_;
  MessageAlloc message_alloc;
  MessageDeleter message_deleter;
};

template<typename Alloc = std::allocator<void>>
class Listener : public rclcpp::Node
{
public:
  Listener(std::string node_name, std::string topic_name, std::shared_ptr<Alloc> alloc)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    rclcpp::SubscriptionOptionsWithAllocator<Alloc> subscription_options;

    subscription_options.allocator = alloc;
    auto msg_mem_strat = std::make_shared<MessageMemoryStrategyT>(alloc);

    auto sub_callback = [&](MessageUniquePtr msg) -> void {
        RCLCPP_INFO(get_logger(), "Subscribed: size() = '%ld'", msg->size());
      };

    sub_ = feature::create_intra_process_subscription<MessageT>(
      this, topic_name, 10, sub_callback, subscription_options, msg_mem_strat);
  }

private:
  typename feature::Subscription<MessageT, Alloc>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto alloc = std::make_shared<Alloc>();

  rclcpp::ExecutorOptions options;
  using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
  using AllocatorMemoryStrategyT = AllocatorMemoryStrategy<Alloc>;
  options.memory_strategy = std::make_shared<AllocatorMemoryStrategyT>(alloc);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);

  auto topic_name = "/chatter";
  auto talker = std::make_shared<Talker<Alloc>>("talker_node", topic_name, alloc);
  auto listener = std::make_shared<Listener<Alloc>>("listener_node", topic_name, alloc);

  exec->add_node(talker);
  exec->add_node(listener);

  try {
    exec->spin();
  } catch (std::bad_alloc & e) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "catched %s", e.what());
  }

  return 0;
}
