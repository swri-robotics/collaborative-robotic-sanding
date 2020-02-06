/*
 * @author ros-industrial
 * @file part_registration.h
 * @date Jan 16, 2020
 * @copyright Copyright (c) 2020, Southwest Research Institute
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#ifndef INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PART_REGISTRATION_MANAGER_H_
#define INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PART_REGISTRATION_MANAGER_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include "crs_application/common/common.h"
#include "crs_application/common/datatypes.h"

namespace crs_application
{
namespace task_managers
{
struct PartRegistrationConfig
{
};

class PartRegistrationManager
{
public:
  PartRegistrationManager(std::shared_ptr<rclcpp::Node> node);
  virtual ~PartRegistrationManager();

  // initialization and configuration
  common::ActionResult init();
  common::ActionResult configure(const PartRegistrationConfig& config);
  common::ActionResult setInput(const datatypes::ScanAcquisitionResult& input);

  // Process Actions
  common::ActionResult computeTransform();
  common::ActionResult showPreview();
  common::ActionResult applyTransform();

  // Results
  const datatypes::ProcessToolpathData& getResult() { return result_; }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<datatypes::ScanAcquisitionResult> input_ = nullptr;
  datatypes::ProcessToolpathData result_;
};

} /* namespace task_managers */
} /* namespace crs_application */

#endif /* INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PART_REGISTRATION_MANAGER_H_ */
