/*
 * @author ros-industrial
 * @file process_execution_manager.h
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

#ifndef INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PROCESS_EXECUTION_MANAGER_H_
#define INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PROCESS_EXECUTION_MANAGER_H_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "crs_application/common/common.h"
#include "crs_application/common/datatypes.h"

namespace crs_application
{
namespace task_managers
{
struct ProcessExecutionConfig
{
};

enum class ProcessExecActions: int
{
  EXEC_PROCESS = 1,
  EXEC_MEDIA_CHANGE,
  DONE
};

class ProcessExecutionManager
{
public:
  ProcessExecutionManager(std::shared_ptr<rclcpp::Node> node);
  virtual ~ProcessExecutionManager();

  // initialization and configuration
  common::ActionResult init();
  common::ActionResult configure(const ProcessExecutionConfig& config);
  common::ActionResult setInput(const datatypes::ProcessExecutionData& input);

  // Process Actions
  /**
   * @brief Checks inputs and other variables and moves to start position if necessary
   * @return True on success, false otherwise
   */
  common::ActionResult moveStart();
  common::ActionResult execProcess();
  common::ActionResult execMediaChange();

  /**
   * @brief moves from media change position back to process
   * @return True on success, false otherwise
   */
  common::ActionResult execMoveReturn();

  /**
   * @brief checks if there are any process motions left in the queue
   * @return  True when the queue is empty
   */
  common::ActionResult checkQueue();

  common::ActionResult execHome();

protected:

  // roscpp
  std::shared_ptr<rclcpp::Node> node_;

  // process data
  std::shared_ptr<datatypes::ProcessExecutionData> input_ = nullptr;

  // other
  int current_process_idx_ = 0;


};

} /* namespace task_managers */
} /* namespace crs_application */

#endif /* INCLUDE_CRS_APPLICATION_TASK_MANAGERS_PROCESS_EXECUTION_MANAGER_H_ */
