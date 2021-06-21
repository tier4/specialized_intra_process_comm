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

#ifndef SPECIALIZED_INTRA_PROCESS__INTRA_PROCESS_MANAGER_HPP_
#define SPECIALIZED_INTRA_PROCESS__INTRA_PROCESS_MANAGER_HPP_

#include <memory>
#include <utility>
#include <vector>
#include <unordered_map>
#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "publisher_wrapper_base.hpp"
#include "typed_subscription_wrapper_base.hpp"

namespace feature
{
class Counter
{
public:
  Counter();
  uint64_t get_incremented_count();

private:
  uint64_t count_;
  std::mutex mutex_;
};

class IntraProcessManager
{
public:
  ~IntraProcessManager();
  uint64_t add_publisher(PublisherBase::SharedPtr publisher);
  uint64_t add_subscription(SubscriptionBase::SharedPtr subscription);
  void remove_subscription(uint64_t intra_process_subscription_id);
  void remove_publisher(uint64_t intra_process_publisher_id);

  template<typename T, typename Alloc>
  using AllocRebind = rclcpp::allocator::AllocRebind<T, Alloc>;

  template<
    typename MessageT,
    typename Alloc = std::allocator<void>
  >
  uint64_t
  do_intra_process_publish(
    uint64_t intra_process_publisher_id,
    std::unique_ptr<MessageT> message,
    std::shared_ptr<typename AllocRebind<MessageT, Alloc>::allocator_type> allocator)
  {
    using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
    using MessageAllocatorT = typename MessageAllocTraits::allocator_type;

    std::shared_lock<std::shared_timed_mutex> lock(mutex_);

    auto topic_name = publishers_[intra_process_publisher_id].topic_name;
    auto seq = sequences_[topic_name]->get_incremented_count();

    auto publisher_it = pub_to_subs_.find(intra_process_publisher_id);
    if (publisher_it == pub_to_subs_.end()) {
      // Publisher is either invalid or no longer exists.
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling do_intra_process_publish for invalid or no longer existing publisher id");
      return seq;
    }
    const auto & sub_ids = publisher_it->second;

    if (sub_ids.take_ownership_subscriptions.empty()) {
      // // None of the buffers require ownership, so we promote the pointer
      std::shared_ptr<MessageT> msg = std::move(message);

      this->template add_shared_msg_to_buffers<MessageT>(
        msg, sub_ids.take_shared_subscriptions, seq);
    } else if (!sub_ids.take_ownership_subscriptions.empty() && // NOLINT
      sub_ids.take_shared_subscriptions.size() <= 1)
    {
      // There is at maximum 1 buffer that does not require ownership.
      // So we this case is equivalent to all the buffers requiring ownership

      // Merge the two vector of ids into a unique one
      std::vector<uint64_t> concatenated_vector(sub_ids.take_shared_subscriptions);
      concatenated_vector.insert(
        concatenated_vector.end(),
        sub_ids.take_ownership_subscriptions.begin(),
        sub_ids.take_ownership_subscriptions.end());

      this->template add_owned_msg_to_buffers<MessageT, Alloc>(
        std::move(message),
        concatenated_vector,
        seq,
        allocator);
    } else if (!sub_ids.take_ownership_subscriptions.empty() && // NOLINT
      sub_ids.take_shared_subscriptions.size() > 1)
    {
      // Construct a new shared pointer from the message
      // for the buffers that do not require ownership
      auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(*allocator, *message);

      this->template add_shared_msg_to_buffers<MessageT>(
        shared_msg, sub_ids.take_shared_subscriptions, seq);
      this->template add_owned_msg_to_buffers<MessageT, Alloc>(
        std::move(message),
        sub_ids.take_ownership_subscriptions,
        seq,
        allocator);
    }
    return seq;
  }

private:
  struct SubscriptionInfo
  {
    SubscriptionInfo() = default;

    SubscriptionBase::WeakPtr subscription;
    rmw_qos_profile_t qos;
    const char * topic_name;
    bool use_take_shared_method;
  };

  struct PublisherInfo
  {
    PublisherInfo() = default;

    PublisherBase::WeakPtr publisher;
    rmw_qos_profile_t qos;
    const char * topic_name;
  };

  struct SplittedSubscriptions
  {
    std::vector<uint64_t> take_shared_subscriptions;
    std::vector<uint64_t> take_ownership_subscriptions;
  };

  using SubscriptionMap =
    std::unordered_map<uint64_t, SubscriptionInfo>;

  using PublisherMap =
    std::unordered_map<uint64_t, PublisherInfo>;

  using PublisherToSubscriptionIdsMap =
    std::unordered_map<uint64_t, SplittedSubscriptions>;

  using SequenceMap =
    std::unordered_map<std::string, std::shared_ptr<Counter>>;

  uint64_t get_next_unique_id();
  bool can_communicate(PublisherInfo pub_info, SubscriptionInfo sub_info) const;
  void insert_sub_id_for_pub(
    uint64_t sub_id, uint64_t pub_id,
    bool use_take_shared_method);

  template<typename MessageT>
  void add_shared_msg_to_buffers(
    std::shared_ptr<const MessageT> message,
    std::vector<uint64_t> subscription_ids,
    uint64_t seq)
  {
    for (auto id : subscription_ids) {
      auto subscription_it = subscriptions_.find(id);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error(
                "subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.subscription;

      using TypedSubscriptionT = feature::TypedSubscriptionBase<MessageT>;
      auto subscription = std::static_pointer_cast<TypedSubscriptionT>(
        subscription_base.lock());

      subscription->provide_intra_process_message(message, seq);
    }
  }

  template<
    typename MessageT,
    typename Alloc = std::allocator<void>
  >
  void
  add_owned_msg_to_buffers(
    std::unique_ptr<MessageT> message,
    std::vector<uint64_t> subscription_ids,
    uint64_t seq,
    std::shared_ptr<typename AllocRebind<MessageT, Alloc>::allocator_type> allocator
  )
  {
    using MessageAllocTraits = AllocRebind<MessageT, Alloc>;
    using MessageUniquePtr = std::unique_ptr<MessageT>;

    // using MessageUniquePtr = std::unique_ptr<MessageT>;

    for (auto it = subscription_ids.begin(); it != subscription_ids.end(); it++) {
      auto subscription_it = subscriptions_.find(*it);
      if (subscription_it == subscriptions_.end()) {
        throw std::runtime_error("subscription has unexpectedly gone out of scope");
      }
      auto subscription_base = subscription_it->second.subscription;

      using TypedSubscriptionT = feature::TypedSubscriptionBase<MessageT>;
      auto subscription = std::static_pointer_cast<TypedSubscriptionT>(
        subscription_base.lock());

      if (std::next(it) == subscription_ids.end()) {
        // ex: provide(message, seq, overwrideseq=true)
        // If this is the last subscription, give up ownership
        subscription->provide_intra_process_message(std::move(message), seq);
      } else {
        // // Copy the message since we have additional subscriptions to serve
        MessageUniquePtr copy_message;
        // Deleter deleter = message.get_deleter();
        // TODO(hsgwa): use message deleter
        auto ptr = MessageAllocTraits::allocate(*allocator.get(), 1);
        MessageAllocTraits::construct(*allocator.get(), ptr, *message);
        copy_message = MessageUniquePtr(ptr);

        subscription->provide_intra_process_message(std::move(copy_message), seq);
      }
    }
  }

  PublisherToSubscriptionIdsMap pub_to_subs_;
  SubscriptionMap subscriptions_;
  PublisherMap publishers_;
  SequenceMap sequences_;
  // TODO(hsgwa): add mutex
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__INTRA_PROCESS_MANAGER_HPP_
