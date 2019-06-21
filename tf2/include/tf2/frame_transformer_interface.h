/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TF2__FRAME_TRANSFORMER_INTERFACE_H_
#define TF2__FRAME_TRANSFORMER_INTERFACE_H_

#include <future>
#include <functional>
#include <string>
#include <vector>

#include "tf2/visibility_control.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

namespace tf2
{

using StampedTransformFuture = std::shared_future<tf2::Stamped<tf2::Transform>>;
using TransformReadyCallback = std::function<void(const StampedTransformFuture&)>;

/**
 * \brief Interface for providing coordinate transforms between any two frames in a system.
 *
 * This class provides a simple abstract interface for looking up relationships between arbitrary
 * frames of a system.
 */
class FrameTransformerInterface
{
public:
  /**
   * \brief Clear internal state data.
   */
  TF2_PUBLIC
  virtual void
  clear() = 0;

  /**
   * \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed.
   * \param source_frame The frame where the data originated.
   * \param time The time at which the value of the transform is desired (0 will get the latest).
   * \return The transform between the frames.
   */
  TF2_PUBLIC
  virtual tf2::Stamped<tf2::Transform>
  getTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time) const = 0;

  /**
   * \brief Get the transform between two frames by frame ID assuming fixed frame.
   * \param target_frame The frame to which data should be transformed.
   * \param target_time The time to which the data should be transformed (0 will get the latest).
   * \param source_frame The frame where the data originated.
   * \param source_time The time at which the source_frame should be evaluated
   *   (0 will get the latest).
   * \param fixed_frame The frame in which to assume the transform is constant in time.
   * \return The transform between the frames.
   */
  TF2_PUBLIC
  virtual tf2::Stamped<tf2::Transform>
  getTransform(
    const std::string& target_frame,
    const tf2::TimePoint& target_time,
    const std::string& source_frame,
    const tf2::TimePoint& source_time,
    const std::string& fixed_frame) const = 0;

  /**
   * \brief Test if a transform is possible.
   * \param target_frame The frame into which to transform.
   * \param source_frame The frame from which to transform.
   * \param time The time at which to transform.
   * \param error_msg A pointer to a string which will be filled with why the transform failed.
   *   Ignored if NULL.
   * \return true if the transform is possible, false otherwise.
   */
  TF2_PUBLIC
  virtual bool
  canTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time,
    std::string* error_msg = NULL) const = 0;

  /**
   * \brief Test if a transform is possible.
   * \param target_frame The frame into which to transform.
   * \param target_time The time into which to transform.
   * \param source_frame The frame from which to transform.
   * \param source_time The time from which to transform.
   * \param fixed_frame The frame in which to treat the transform as constant in time.
   * \param error_msg A pointer to a string which will be filled with why the transform failed.
   *   Ignored if NULL.
   * \return true if the transform is possible, false otherwise.
   */
  TF2_PUBLIC
  virtual bool
  canTransform(
    const std::string& target_frame,
    const tf2::TimePoint& target_time,
    const std::string& source_frame,
    const tf2::TimePoint& source_time,
    const std::string& fixed_frame,
    std::string* error_msg = NULL) const = 0;

  /**
   * \brief Get frames that exist in the system.
   */
  // TF2_PUBLIC
  // virtual std::vector<std::string>
  // getFrames() const = 0;

  /**
   * \brief Wait for a transform between two frames to become available.
   * \param target_frame The frame into which to transform.
   * \param source_frame The frame from which to tranform.
   * \param time The time at which to transform.
   * \param timeout Duration after which waiting will be stopped.
   * \param callback The function to be called when the transform becomes available or a timeout
   *   occurs. In the case of timeout, an exception will be set on the future.
   * \return A future to the requested transform. If a timeout occurs an exception will be set on
   *   the future.
   */
  TF2_PUBLIC
  virtual StampedTransformFuture
  waitForTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2::TimePoint& time,
    const tf2::Duration& timeout,
    TransformReadyCallback callback) = 0;
};  // class FrameTransformerInterface

}  // namespace tf2

#endif // TF2__FRAME_TRANSFORMER_INTERFACE_H_
