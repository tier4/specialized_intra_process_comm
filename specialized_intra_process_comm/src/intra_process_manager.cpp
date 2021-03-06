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

#include <unordered_map>
#include <memory>
#include <utility>

#include "specialized_intra_process_comm/intra_process_manager.hpp"

namespace feature
{

Counter::Counter()
: count_(0)
{
}

uint64_t Counter::get_incremented_count()
{
  std::lock_guard<std::mutex> lock(mutex_);
  count_++;
  return count_;
}

static std::atomic<uint64_t> _next_unique_id{1};

IntraProcessManager::~IntraProcessManager() {}

uint64_t IntraProcessManager::add_publisher(PublisherBase::SharedPtr publisher)
{
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  auto id = IntraProcessManager::get_next_unique_id();

  publishers_[id].publisher = publisher;
  publishers_[id].topic_name = publisher->get_topic_name();
  publishers_[id].qos = publisher->get_actual_qos().get_rmw_qos_profile();

  // Initialize the subscriptions storage for this publisher.
  pub_to_subs_[id] = SplittedSubscriptions();
  auto topic_name = publishers_[id].topic_name;
  auto has_key = sequences_.find(topic_name) != sequences_.end();
  if (!has_key) {
    sequences_[topic_name] = std::make_shared<feature::Counter>();
  }

  // create an entry for the publisher id and populate with already existing
  // subscriptions
  for (auto & pair : subscriptions_) {
    if (can_communicate(publishers_[id], pair.second)) {
      insert_sub_id_for_pub(pair.first, id, pair.second.use_take_shared_method);
    }
  }

  return id;
}

void IntraProcessManager::insert_sub_id_for_pub(
  uint64_t sub_id, uint64_t pub_id, bool use_take_shared_method)
{
  if (use_take_shared_method) {
    pub_to_subs_[pub_id].take_shared_subscriptions.push_back(sub_id);
  } else {
    pub_to_subs_[pub_id].take_ownership_subscriptions.push_back(sub_id);
  }
}

uint64_t IntraProcessManager::add_subscription(SubscriptionBase::SharedPtr subscription)
{
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  auto id = IntraProcessManager::get_next_unique_id();

  subscriptions_[id].subscription = subscription;
  subscriptions_[id].topic_name = subscription->get_topic_name();
  subscriptions_[id].qos = subscription->get_actual_qos().get_rmw_qos_profile();
  subscriptions_[id].use_take_shared_method = subscription->use_take_shared_method();

  // adds the subscription id to all the matchable publishers
  for (auto & pair : publishers_) {
    if (can_communicate(pair.second, subscriptions_[id])) {
      insert_sub_id_for_pub(id, pair.first, subscriptions_[id].use_take_shared_method);
    }
  }

  return id;
}

void IntraProcessManager::remove_subscription(uint64_t intra_process_subscription_id)
{
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  subscriptions_.erase(intra_process_subscription_id);

  for (auto & pair : pub_to_subs_) {
    pair.second.take_shared_subscriptions.erase(
      std::remove(
        pair.second.take_shared_subscriptions.begin(), pair.second.take_shared_subscriptions.end(),
        intra_process_subscription_id),
      pair.second.take_shared_subscriptions.end());

    pair.second.take_ownership_subscriptions.erase(
      std::remove(
        pair.second.take_ownership_subscriptions.begin(),
        pair.second.take_ownership_subscriptions.end(), intra_process_subscription_id),
      pair.second.take_ownership_subscriptions.end());
  }
}

void IntraProcessManager::remove_publisher(uint64_t intra_process_publisher_id)
{
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  publishers_.erase(intra_process_publisher_id);
  pub_to_subs_.erase(intra_process_publisher_id);
}

uint64_t IntraProcessManager::get_next_unique_id()
{
  auto next_id = _next_unique_id.fetch_add(1, std::memory_order_relaxed);
  // Check for rollover (we started at 1).
  if (0 == next_id) {
    // This puts a technical limit on the number of times you can add a publisher or subscriber.
    // But even if you could add (and remove) them at 1 kHz (very optimistic rate)
    // it would still be a very long time before you could exhaust the pool of id's:
    //   2^64 / 1000 times per sec / 60 sec / 60 min / 24 hours / 365 days = 584,942,417 years
    // So around 585 million years. Even at 1 GHz, it would take 585 years.
    // I think it's safe to avoid trying to handle overflow.
    // If we roll over then it's most likely a bug.
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::overflow_error(
      "exhausted the unique id's for publishers and subscribers in this process "
      "(congratulations your computer is either extremely fast or extremely old)");
    // *INDENT-ON*
  }
  return next_id;
}

bool IntraProcessManager::can_communicate(PublisherInfo pub_info, SubscriptionInfo sub_info) const
{
  // publisher and subscription must be on the same topic
  if (strcmp(pub_info.topic_name, sub_info.topic_name) != 0) {
    return false;
  }

  // TODO(alsora): the following checks for qos compatibility should be provided
  // by the RMW a reliable subscription can't be connected with a best effort
  // publisher
  if (
    sub_info.qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
    pub_info.qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
  {
    return false;
  }

  // a publisher and a subscription with different durability can't communicate
  if (sub_info.qos.durability != pub_info.qos.durability) {
    return false;
  }

  return true;
}
}  // namespace feature
