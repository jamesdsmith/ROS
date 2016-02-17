/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Start up a new message synchronizer.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef MESSAGE_SYNCHRONIZER_H
#define MESSAGE_SYNCHRONIZER_H

#include <vector>
#include <algorithm>
#include <functional>

template<typename MessageType>
class MessageSynchronizer {
 public:
  explicit MessageSynchronizer();
  ~MessageSynchronizer();

  bool Initialize(const ros::NodeHandle& n, const std::string& topic,
                  double timer_period);


  // Add message.
  void AddMessage(const MessageType& msg);
  void GetSorted(std::vector<MessageType>& sorted);

  // Member variables.
  std::vector<MessageType> buffer_;
};

// ------------------------- IMPLEMENTATION ---------------------------------- //

// Constructor/destructor.
template<typename MessageType>
MessageSynchronizer<MessageType>::MessageSynchronizer() {}

template<typename MessageType>
MessageSynchronizer<MessageType>::~MessageSynchronizer() {}

// Message callback.
template<typename MessageType>
void MessageSynchronizer<MessageType>::AddMessage(const MessageType& msg) {
  buffer_.push_back(msg);
}

template<typename MessageType>
struct TimeComparitor {
  bool operator()(const MessageType& msg1, const MessageType& msg2) {
    return msg1->header.stamp < msg2->header.stamp;
  }
};

// Timer callback.
template<typename MessageType>
void MessageSynchronizer<MessageType>::GetSorted(std::vector<MessageType>& sorted) {
  // Sort buffer_ by timestamps.
  std::sort(buffer_.begin(), buffer_.end(), TimeComparitor<MessageType>());

  // Copy into a new vector.
  sorted.clear();
  for (size_t ii = 0; ii < buffer_.size(); ii++)
    sorted.push_back(buffer_[ii]);

  // Clear buffer_.
  buffer_.clear();
}

#endif
