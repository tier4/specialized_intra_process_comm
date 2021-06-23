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

#include "specialized_intra_process/subscription_base.hpp"

#include "specialized_intra_process/intra_process_manager.hpp"

namespace feature
{
SubscriptionBase::SubscriptionBase() {}

void SubscriptionBase::post_init_setup(rclcpp::SubscriptionBase::SharedPtr sub) {sub_base_ = sub;}

SubscriptionBase::~SubscriptionBase()
{
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    RCLCPP_WARN(
      rclcpp::get_logger("feature"), "Intra process manager died before than a subscription.");
    return;
  }
  ipm->remove_subscription(intra_process_subscription_id_);
}

const char * SubscriptionBase::get_topic_name() const {return sub_base_->get_topic_name();}

rclcpp::QoS SubscriptionBase::get_actual_qos() const {return sub_base_->get_actual_qos();}

void SubscriptionBase::setup_intra_process(
  uint64_t intra_process_subscription_id, IntraProcessManagerWeakPtr weak_ipm)
{
  intra_process_subscription_id_ = intra_process_subscription_id;
  weak_ipm_ = weak_ipm;
}
}  // namespace feature
