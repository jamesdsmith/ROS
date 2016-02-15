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

#include <message_synchronizer/message_synchronizer.h>

// Constructor/destructor.
template<typename MessageType>
MessageSynchronizer<MessageType>::MessageSynchronizer() {}

template<typename MessageType>
MessageSynchronizer<MessageType>::~MessageSynchronizer() {}

// Initialize.
template<typename MessageType>
bool MessageSynchronizer<MessageType>::Initialize(const ros::NodeHandle& n,
                                                  const std::string& topic,
                                                  double timer_period) {
  name_ = ros::names::append(n.getNamespace(), "message_synchronizer");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n, topic, timer_period)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

// Load parameters.
template<typename MessageType>
bool MessageSynchronizer<MessageType>::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callback functions.
template<typename MessageType>
bool MessageSynchronizer<MessageType>::RegisterCallbacks(const ros::NodeHandle& n,
                                                         const std::string& topic,
                                                         double timer_period) {
  ros::NodeHandle node(n);

  subscriber_ =
    node.subscribe<MessageType>(topic.c_str(), 10,
                                &MessageSynchronizer::AddMessageCallback, this);
  publisher_ = node.advertise<MessageType>(topic.c_str(), 10, false);
  timer_ = n.createTimer(ros::Duration(timer_period),
                         &MessageSynchronizer::TimerCallback, this);

  return true;
}

// Message callback.
template<typename MessageType>
void MessageSynchronizer<MessageType>::AddMessageCallback(const MessageType& msg) {
  buffer_.push_back(msg);
}

// Timer callback.
template<typename MessageType>
void MessageSynchronizer<MessageType>::TimerCallback() {
  // Create a custom comparitor for sorting by timestamp.
  struct {
    bool operator()(MessageType& msg1, MessageType& msg2) {
      return msg1.header.stamp < msg2.header.stamp;
    }
  } time_comparitor;

  // Sort buffer_ by timestamps.
  std::sort(buffer_.begin(), buffer_.end(), time_comparitor);

  // Publish each message in order.
  for (size_t ii = 0; ii < buffer_.size(); ii++) {
    MessageType& msg = buffer_[ii];
    publisher_.publish(msg);
  }

  // Clear buffer_.
  buffer_.clear();
}
