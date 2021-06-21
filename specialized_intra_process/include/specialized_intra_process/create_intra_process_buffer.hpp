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


#ifndef SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_BUFFER_HPP_
#define SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_BUFFER_HPP_

#include <memory>
#include <utility>

#include "buffers/intra_process_buffer.hpp"
#include "buffers/ring_buffer_implementation.hpp"
#include "intra_process_buffer_type.hpp"

namespace feature
{
template<
  typename MessageT,
  typename IntraProcessBufferT = feature::buffers::IntraProcessBuffer<MessageT>,
  typename Alloc = std::allocator<void>>
typename IntraProcessBufferT::UniquePtr create_intra_process_buffer(
  IntraProcessBufferType buffer_type, rmw_qos_profile_t qos)
{
  using MessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT>;

  size_t buffer_size = qos.depth;
  typename IntraProcessBufferT::UniquePtr buffer;

  switch (buffer_type) {
    case IntraProcessBufferType::SharedPtr: {
        using BufferT = MessageSharedPtr;
        using BufferImplementationT = feature::buffers::RingBufferImplementation<BufferT>;

        auto buffer_implementation = std::make_unique<BufferImplementationT>(buffer_size);

        // Construct the intra_process_buffer
        using TypedIntraProcessBufferT =
          feature::buffers::TypedIntraProcessBuffer<MessageT, Alloc, BufferT>;
        buffer = std::make_unique<TypedIntraProcessBufferT>(
          std::move(buffer_implementation));

        break;
      }
    case IntraProcessBufferType::UniquePtr: {
        using BufferT = MessageUniquePtr;
        using BufferImplementationT = feature::buffers::RingBufferImplementation<BufferT>;
        auto buffer_implementation = std::make_unique<BufferImplementationT>(buffer_size);

        // Construct the intra_process_buffer
        using TypedIntraProcessBufferT = feature::buffers::TypedIntraProcessBuffer<MessageT, Alloc,
            BufferT>;
        buffer = std::make_unique<TypedIntraProcessBufferT>(std::move(buffer_implementation));
        break;
      }
    default: {
        break;
      }
  }
  return buffer;
}
}  // namespace feature

#endif  // SPECIALIZED_INTRA_PROCESS__CREATE_INTRA_PROCESS_BUFFER_HPP_
