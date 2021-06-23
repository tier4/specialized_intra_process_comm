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

#ifndef SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_BASE_HPP_
#define SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_BASE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace feature
{
/**
 * IntraProcessManager is forward declared here, avoiding a circular inclusion
 * between `intra_process_manager.hpp` and `subscription_base.hpp`.
 */
class IntraProcessManager;

class SubscriptionBase : public std::enable_shared_from_this<SubscriptionBase>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionBase)

  SubscriptionBase();

  void post_init_setup(rclcpp::SubscriptionBase::SharedPtr sub);

  /// Default destructor.
  virtual ~SubscriptionBase();

  virtual bool use_take_shared_method() const = 0;

  const char * get_topic_name() const;

  rclcpp::QoS get_actual_qos() const;

  using IntraProcessManagerWeakPtr = std::weak_ptr<feature::IntraProcessManager>;

  void setup_intra_process(
    uint64_t intra_process_subscription_id, IntraProcessManagerWeakPtr weak_ipm);

  IntraProcessManagerWeakPtr weak_ipm_;
  uint64_t intra_process_subscription_id_;
  std::shared_ptr<rclcpp::SubscriptionBase> sub_base_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_BASE_HPP_
