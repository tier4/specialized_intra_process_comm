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

#include "specialized_intra_process/publisher_base.hpp"

#include "specialized_intra_process/intra_process_manager.hpp"

namespace feature
{
PublisherBase::PublisherBase(rclcpp::PublisherBase::SharedPtr pub)
: pub_base_(pub) {}

PublisherBase::~PublisherBase()
{
  // must fini the events before fini-ing the publisher

  auto ipm = weak_ipm_.lock();

  if (!ipm) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Intra process manager died before a publisher.");
    return;
  }
  ipm->remove_publisher(intra_process_publisher_id_);
}

const char * PublisherBase::get_topic_name() const {return pub_base_->get_topic_name();}

rclcpp::QoS PublisherBase::get_actual_qos() const {return pub_base_->get_actual_qos();}

void PublisherBase::setup_intra_process(
  uint64_t intra_process_publisher_id, IntraProcessManagerSharedPtr ipm)
{
  intra_process_publisher_id_ = intra_process_publisher_id;
  weak_ipm_ = ipm;
}
}  // namespace feature
