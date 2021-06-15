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

#ifndef SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_WRAPPER_HPP_
#define SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_WRAPPER_HPP_

#include <memory>

#include "create_intra_process_buffer.hpp"
#include "intra_process_buffer_type.hpp"
#include "intra_process_manager.hpp"
#include "notification_msgs/msg/notification.hpp"
#include "rclcpp/rclcpp.hpp"
#include "typed_subscription_wrapper_base.hpp"

namespace feature
{
template<
  typename CallbackMessageT,
  typename TypedSubscriptionBaseT = TypedSubscriptionBase<CallbackMessageT>>
class Subscription : public TypedSubscriptionBaseT
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription)

  using NotifySubscriptionT = typename rclcpp::Subscription<notification_msgs::msg::Notification>;

  using BufferSharedPtr =
    typename feature::buffers::IntraProcessBuffer<CallbackMessageT>::SharedPtr;
  using MessageUniquePtr = std::unique_ptr<CallbackMessageT>;

  Subscription()
  : TypedSubscriptionBaseT() {}

  ~Subscription() {}

  void post_init_setup(rclcpp::Node * node, NotifySubscriptionT::SharedPtr sub)
  {
    TypedSubscriptionBase<CallbackMessageT>::post_init_setup(sub);
    sub_ = sub;

    auto node_base = node->get_node_base_interface();
    auto context = node_base->get_context();
    auto ipm = context->get_sub_context<feature::IntraProcessManager>();

    uint64_t intra_process_subscription_id = ipm->add_subscription(this->shared_from_this());
    this->setup_intra_process(intra_process_subscription_id, ipm);
  }

  // private:
  NotifySubscriptionT::SharedPtr sub_;
};
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__SUBSCRIPTION_WRAPPER_HPP_
