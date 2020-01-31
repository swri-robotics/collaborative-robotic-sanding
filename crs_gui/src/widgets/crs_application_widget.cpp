/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ui_crs_application.h"

#include <atomic>
#include <QMessageBox>
#include <QStateMachine>
#include <QProgressBar>
#include <QProgressDialog>

#include <crs_gui/widgets/crs_application_widget.h>

#include <crs_gui/widgets/part_selection_widget.h>
//#include "application_state_properties_impl.h"
//#include <crs_application/application/application_context_base.h>
//#include <crs_gui/application_state_machine.h>
//#include <crs_gui/widgets/application_widget_base.h>
//#include <crs_gui/widgets/database_selection_widget.h>
//#include <crs_gui/widgets/trajectory_approval_widget.h>
//#include <crs_gui/widgets/manual_manipulation_widget.h>
//#include <crs_gui/widgets/database_log_widget.h>
//#include <crs_msgs/LetUserJogRobot.h>

const static std::string TRAJECTORY_PREVIEW_TOPIC = "trajectory_preview";
const static std::string STATE_PREVIEW_TOPIC = "state_preview";
const static std::string APPROVAL_ACTION = "trajectory_approval";

const static std::string ALLOW_OPERATOR_CONTROL_SERVICE = "allow_operator_control";

namespace crs_gui
{

CRSApplicationWidget::CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                                       QWidget* parent,
                                       std::string database_directory)
  : QWidget(parent), ui_(new Ui::CRSApplication), part_selector_widget_(new PartSelectionWidget())
//  , user_control_action_server_(ALLOW_OPERATOR_CONTROL_SERVICE, boost::bind(&CRSApplicationWidget::userControlCallback, this, _1), false)
//  , app_(app)
{
  ui_->setupUi(this);

//  // Set the application callback functions
//  app_->setRemovePartCallback(boost::bind(&CRSApplicationWidget::onPartRemoved, this, _1));
//  app_->setActivePartSetCallback(boost::bind(&CRSApplicationWidget::onActivePartSet, this, _1, _2));

//  // Create the widgets
//  part_selector_widget_ = new PartSelectorWidget(app_, center_of_workcell, this);
//  job_selector_widget_ = new JobSelectorWidget(app_, this);
//  trajectory_approval_widget_ = new TrajectoryApprovalWidget(TRAJECTORY_PREVIEW_TOPIC,
//                                                                     STATE_PREVIEW_TOPIC,
//                                                                     APPROVAL_ACTION, this);

//  manual_manipulation_widget_ = new ManualManipulationWidget();

//  database_log_widget_ = new DatabaseLogWidget(app_, this);

//  progress_bar_ = new QProgressBar(this);
//  progress_bar_->setRange(0, 100);
//  progress_bar_->setValue(0);

//  // Add the widgets to the UI
  ui_->vertical_layout_part_selector->addWidget(part_selector_widget_);
//  ui_->vertical_layout_job_selector->addWidget(job_selector_widget_);
//  ui_->vertical_layout_preview_approval->addWidget(trajectory_approval_widget_);
//  ui_->vertical_layout_localize->addWidget(progress_bar_);
//  ui_->vertical_layout_manipulation->addWidget(manual_manipulation_widget_);
//  ui_->vertical_layout_log->addWidget(database_log_widget_);
  
//  // Create the state machine to manage the UI behavior
//  createStateMachine();

//  // Connect the signals and slots
//  connect(trajectory_approval_widget_, &TrajectoryApprovalWidget::recievedTrajectory, this, &CRSApplicationWidget::showTrajectory);
//  connect(trajectory_approval_widget_, &TrajectoryApprovalWidget::approved, this, &CRSApplicationWidget::hideTrajectory);
//  connect(trajectory_approval_widget_, &TrajectoryApprovalWidget::rejected, this, &CRSApplicationWidget::hideTrajectory);

//  connect(manual_manipulation_widget_, &ManualManipulationWidget::showWarningDialog, this, &CRSApplicationWidget::onShowWarningDialog);

//  connect(manual_manipulation_widget_, &ManualManipulationWidget::reset, this, &CRSApplicationWidget::resetToCurrentStateManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::show, this, &CRSApplicationWidget::showManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::hide, this, &CRSApplicationWidget::hideManipulationWidget);

//  connect(manual_manipulation_widget_, &ManualManipulationWidget::showCartesianManipulation, this, &CRSApplicationWidget::showCartesianManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::hideCartesianManipulation, this, &CRSApplicationWidget::hideCartesianManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::showJointManipulation, this, &CRSApplicationWidget::showJointManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::hideJointManipulation, this, &CRSApplicationWidget::hideJointManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::manipulatorChanged, this, &CRSApplicationWidget::manipulatorChangedManipulationWidget);
//  connect(manual_manipulation_widget_, &ManualManipulationWidget::tcpLinkChanged, this, &CRSApplicationWidget::tcpLinkChangedManipulationWidget);

  connect(part_selector_widget_, &PartSelectionWidget::partSelected, this, &CRSApplicationWidget::onPartSelected);
//  connect(part_selector_widget_, &PartSelectorWidget::okToProceed, this, &CRSApplicationWidget::onPartSelectionStatusChanged);
//  connect(job_selector_widget_, &JobSelectorWidget::okToProceed, this, &CRSApplicationWidget::onJobSelectionStatusChanged);

//  connect(ui_->push_button_scan_booth, &QPushButton::clicked, this, &CRSApplicationWidget::onScanBooth);
//  connect(ui_->push_button_align_to_scan, &QPushButton::clicked, this, &CRSApplicationWidget::onAlignToScan);
//  connect(ui_->push_button_start_localization, &QPushButton::clicked, this, &CRSApplicationWidget::onStartLocalization);
//  connect(ui_->push_button_verify_localization, &QPushButton::clicked, this, &CRSApplicationWidget::onStartLocalizationVerification);
//  connect(ui_->push_button_load_localization, &QPushButton::clicked, this, &CRSApplicationWidget::onLoadLocLog);
//  connect(ui_->push_button_detailed_scan, &QPushButton::clicked, this, &CRSApplicationWidget::onDetailedScanLoc);

//  connect(ui_->push_button_plan, &QPushButton::clicked, this, &CRSApplicationWidget::onPlanJob);
//  connect(ui_->push_button_execute, &QPushButton::clicked, this, &CRSApplicationWidget::onExecuteJob);
//  connect(ui_->push_button_safe_go_home, &QPushButton::clicked, this, &CRSApplicationWidget::onSafelyGoHome);
//  connect(ui_->push_button_cancel_current_task, &QPushButton::clicked, this, &CRSApplicationWidget::onCancelCurrentTask);

//  connect(this, &CRSApplicationWidget::planComplete, this, &CRSApplicationWidget::onApproveJob);
//  connect(this, &CRSApplicationWidget::showWarningDialog, this, &CRSApplicationWidget::onShowWarningDialog);
//  connect(this, &CRSApplicationWidget::showInfoDialog, this, &CRSApplicationWidget::onShowInfoDialog);
//  connect(this, &CRSApplicationWidget::updateProgressBar, this, &CRSApplicationWidget::onUpdateProgressBar);

//  // Start the user control action server
//  user_control_action_server_.start();
}

void CRSApplicationWidget::createStateMachine()
{
//  sm_ = new QStateMachine(this);

  /* Create the states of the state machine.
   *
   * The states below are defined in sequential order, and a given state's parent should be the
   * previously defined state's meta-state (or the state machine in the case of the first state)
   *
   * Note: Seeing weird behavior when using QPushButton clicked signal to trigger state machine transition.
   * Better behavior when using pressed signal instead
   */
//  NestedState select_part ("select_part",
//                           sm_,
//                           this, SIGNAL(noPartSelected()));

//  NestedState localization_ready ("localization_ready",
//                                  select_part.meta_state,
//                                  this, SIGNAL(partSelected()));

//  NestedState localization_started ("localization_started",
//                                    localization_ready.meta_state,
//                                    ui_->push_button_start_localization, SIGNAL(released()));

//  NestedState verification_ready ("verification_ready",
//                                  localization_started.meta_state,
//                                  this, SIGNAL(partLocalized()));

//  NestedState verification_started ("verification_started",
//                                    verification_ready.meta_state,
//                                    ui_->push_button_verify_localization, SIGNAL(released()));

//  NestedState verification_complete ("verification_complete",
//                                     verification_started.meta_state,
//                                     this, SIGNAL(partLocalizationVerified()));

//  NestedState select_job ("select_job",
//                          verification_complete.meta_state,
//                          this, SIGNAL(noJobSelected()));

//  NestedState planning_ready("planning_ready",
//                             select_job.meta_state,
//                             this, SIGNAL(jobSelected()));

//  NestedState planning_started ("planning_started",
//                                planning_ready.meta_state,
//                                ui_->push_button_plan, SIGNAL(released()));

//  NestedState approve_trajectory("approve_trajectory",
//                                 planning_started.meta_state,
//                                 this, SIGNAL(planComplete()));

//  NestedState execution_ready ("execution_ready",
//                               approve_trajectory.meta_state,
//                               this, SIGNAL(jobApproved()));

//  NestedState execution_started ("execution_started",
//                                 execution_ready.meta_state,
//                                 ui_->push_button_execute, SIGNAL(released()));

//  NestedState execution_complete ("execution_complete",
//                                  execution_started.meta_state,
//                                  this, SIGNAL(executionComplete()));

//  // Since crs_4702 does not currently have detailed scan capability,
//  // check whether we are on that robot.  This result will be used to
//  // enable or disable the detailed scan button.
//  ROS_ERROR_STREAM(app_->getRobotId().c_str());
//  bool use_detailed_scan = (app_->getRobotId() != std::string("crs_4702"));
//  ROS_ERROR_STREAM(use_detailed_scan);

//  // Assign properties to the UI for each state (i.e. buttons/tabs enabled, colors, etc.)
//  assignPropertiesSelectPartState(select_part.state, ui_, use_detailed_scan);
//  assignPropertiesLocalizationReadyState(localization_ready.state, ui_, use_detailed_scan);
//  assignPropertiesLocalizationStartedState(localization_started.state, ui_, use_detailed_scan);
//  assignPropertiesVerificationReadyState(verification_ready.state, ui_, use_detailed_scan);
//  assignPropertiesVerificationStartedState(verification_started.state, ui_, use_detailed_scan);
//  assignPropertiesVerificationCompleteState(verification_complete.state, ui_, use_detailed_scan);
//  assignPropertiesSelectJobState(select_job.state, ui_, use_detailed_scan);
//  assignPropertiesPlanningReadyState(planning_ready.state, ui_, use_detailed_scan);
//  assignPropertiesPlanningStartedState(planning_started.state, ui_, use_detailed_scan);
//  assignPropertiesApproveTrajectoryState(approve_trajectory.state, ui_, use_detailed_scan);
//  assignPropertiesExecutionReadyState(execution_ready.state, ui_, use_detailed_scan);
//  assignPropertiesExecutionStartedState(execution_started.state, ui_, use_detailed_scan);
//  assignPropertiesExecutionCompleteState(execution_complete.state, ui_, use_detailed_scan);

//  // Create a queue of states, ordered in the correct sequence from first to last
//  std::queue<NestedState> states;
//  states.push(select_part);
//  states.push(localization_ready);
//  states.push(localization_started);
//  states.push(verification_ready);
//  states.push(verification_started);
//  states.push(verification_complete);
//  states.push(select_job);
//  states.push(planning_ready);
//  states.push(planning_started);
//  states.push(approve_trajectory);
//  states.push(execution_ready);
//  states.push(execution_started);
//  states.push(execution_complete);

//  createNestedSequentialStateMachine(sm_, states);

//  sm_->setInitialState(select_part.meta_state);
//  sm_->start();
}

//void CRSApplicationWidget::userControlCallback(const crs_msgs::AllowUserControlGoalConstPtr& goal)
//{
  // Create a thread-safe variable to keep track of the state of the window
//  std::atomic<bool> done (false);

//  // Create an inline "slot" that sets the done flag to true when the dialog window is closed
//  connect(this, &CRSApplicationWidget::warningDialogClosed, [&done]()
//  {
//    done = true;
//  });

//  /* Since this method will be called by the ROS thread (because it is a ROS service callback), emit a signal
//   * to have the GUI thread bring up a dialog box */
//  emit showWarningDialog("User Drive", "You may now drive the robot manually. Close this window when you are finished moving the robot.\n\nPoint Description:\n" + QString::fromStdString(goal->message));

//  // Wait for the done flag to get set true
//  while(!done)
//  {
//    ROS_DEBUG_STREAM_THROTTLE(2.0, "Waiting for user to relinquish control of the robot...");
//  }

//  crs_msgs::AllowUserControlResult result;
//  result.success = true;
//  result.message = "Done";

//  user_control_action_server_.setSucceeded(result);
//}


void CRSApplicationWidget::onPartSelected(const std::string selected_part)
{
  std::cout << selected_part << std::endl;
}


void CRSApplicationWidget::onPartSelectionStatusChanged(const bool status)
{

////  else  if(status)
  {
    emit partSelected();
  }
//  {
//    emit noPartSelected();
//  }

  // TODO: update the job selection widget to already have the right part selected (and others disabled?)
}

void CRSApplicationWidget::onPartRemoved(const bool success)
{
  emit noPartSelected();
  if(!success)
  {
    emit showWarningDialog("Part Removed", "Failed to remove selected part from the planning environment");
  }
}

void CRSApplicationWidget::onSafelyGoHome()
{
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onSafelyWentHome, this, _1);

//  crs_application::TaskResponse res = app_->safelyGoHome(notify_cb);
//  if (!res.success)
//  {
//    QMessageBox::warning(this, "Retreat to Home Error", QString::fromStdString(res.message));
////    emit partSelected(); // Send to the simplest state that includes a part collision model
//    return;
//  }
}

void CRSApplicationWidget::onSafelyWentHome(const bool success)
{
  if(!success)
  {
//    emit partSelected(); // Send to the simplest state that includes a part collision model
    emit showWarningDialog("Retreat to Home Error", "Failed to safely go home");
  }
  else
  {
//    emit partSelected(); // Send to the simplest state that includes a part collision model
    emit showInfoDialog("Retreat to Home", "Safely sent robot home");
  }
}

void CRSApplicationWidget::onScanBooth()
{
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onScanBoothComplete, this, _1);

//  crs_application::TaskResponse res = app_->scanBooth(notify_cb);
//  if (!res.success)
//  {
//    QMessageBox::warning(this, "Booth Scan Error", QString::fromStdString(res.message));
//  }
//  return;
}

void CRSApplicationWidget::onScanBoothComplete(const bool success)
{
  if (success)
  {
    emit showInfoDialog("Booth Scan", "Completed Booth Scan");
  }
  else
  {
    emit showWarningDialog("Booth Scan", "Booth Scan Failed");
  }
  return;
}

void CRSApplicationWidget::onAlignToScan()
{
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationComplete, this, _1);

//  crs_application::TaskResponse res = app_->alignToScan(notify_cb);
//  if (!res.success)
//  {
//    QMessageBox::warning(this, "Alignment to Scan Error", QString::fromStdString(res.message));
//  }
//  else {
//    emit updateProgressBar(50);
//  }
//  return;
}

void CRSApplicationWidget::onStartLocalization()
{
//  QMessageBox::StandardButton reply;

//  reply = QMessageBox::question(this, "Proceed", "Are you sure you want to manually localize part?", QMessageBox::Yes | QMessageBox::No);
//  if(reply == QMessageBox::No)
//  {
//    emit partSelected();
//    return;
//  }
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationComplete, this, _1);

//  crs_application::PercentageFeedbackCBType feedback_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationFeedback, this, _1);

//  crs_application::TaskResponse res = app_->localizePart(notify_cb, feedback_cb);
//  if(!res.success)
//  {
//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//    emit partSelected();
//  }
}

void CRSApplicationWidget::onLoadLocLog()
{
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationComplete, this, _1);

//  crs_application::TaskResponse res = app_->getLocLog(notify_cb);
//  if(!res.success)
//  {
//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//    emit partSelected();
//  }
}

void CRSApplicationWidget::onDetailedScanLoc()
{
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationComplete, this, _1);

//  crs_application::TaskResponse res = app_->detailedScanLoc(notify_cb);
//  if(!res.success)
//  {
//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//    emit partSelected();
//  }
}

void CRSApplicationWidget::onLocalizationFeedback(const int pct_complete)
{
  emit updateProgressBar(static_cast<int>(pct_complete));
}

void CRSApplicationWidget::onLocalizationComplete(const bool success)
{
  emit updateProgressBar(100);
  if(!success)
  {
    emit partSelected();
    emit showWarningDialog("Localization Error", "Failed to localize part");
  }
  else
  {
    emit partLocalized();
    emit showInfoDialog("Localization Complete", "Part successfully localized");
  }

  emit updateProgressBar(0);
}

void CRSApplicationWidget::onStartLocalizationVerification()
{
//  crs_application::NotifyCBType notify_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationVerificationComplete, this, _1);

//  crs_application::PercentageFeedbackCBType feedback_cb =
//      boost::bind(&CRSApplicationWidget::onLocalizationVerificationFeedback, this, _1);

//  crs_application::TaskResponse res = app_->verifyPartLocalization(notify_cb, feedback_cb);
//  if(!res.success)
//  {
//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//    emit partLocalized();
//  }
}

void CRSApplicationWidget::onLocalizationVerificationFeedback(const int pct_complete)
{
  emit updateProgressBar(static_cast<int>(pct_complete));
}

void CRSApplicationWidget::onLocalizationVerificationComplete(const bool success)
{
  if(!success)
  {
    emit partLocalized();
    emit showWarningDialog("Localization Verification Error", "Failed to verify part localization");
  }
  else
  {
    emit partLocalizationVerified();
    emit showInfoDialog("Localization Verification Complete", "Successfully verified part localization");
//    job_selector_widget_->onPartSelect();
  }

  emit updateProgressBar(0);
}

void CRSApplicationWidget::onJobSelectionStatusChanged(const bool status)
{
  if(status)
  {
    emit jobSelected();
  }
}

void CRSApplicationWidget::onPlanJob()
{
  // Add a progress bar to let the user know something is happening
  progress_dialog_ = new QProgressDialog(this);
  progress_dialog_->setModal(true);
  progress_dialog_->setLabelText("Motion Planning Progress");
  progress_dialog_->setMinimum(0);
  progress_dialog_->setMaximum(100);
  progress_dialog_->setValue(progress_dialog_->minimum());
  progress_dialog_->setCancelButton(nullptr);   // TODO: Connect to cancel action
  progress_dialog_->hide();

//  crs_application::TaskResponse res = app_->planJob(boost::bind(&CRSApplicationWidget::onPlanJobComplete, this, _1));
//  if(!res.success)
//  {
//    emit jobSelected();
//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//  }
//  else {
//    progress_dialog_->show();
//  }
}

void CRSApplicationWidget::onPlanJobComplete(const bool success)
{
  // Be really careful about null pointers here, as it is easy for someone to 'x out' of the popup
  int i = 0;
  if (progress_dialog_ != nullptr)
  {
    i = progress_dialog_->value();
  }
  for (/*i already declared*/; progress_dialog_ != nullptr && i < progress_dialog_->maximum(); ++i)
  {
    progress_dialog_->setValue(i);
//    ros::Duration(0.01).sleep();
  }
  if (progress_dialog_ != nullptr)
  {
    progress_dialog_->hide();
  }

  if(!success)
  {
    emit jobSelected();
    emit showWarningDialog("Process Plan Error", "Failed to create process plan for selected job");
  }
  else
  {
    emit planComplete();
    emit showInfoDialog("Process Plan Complete", "Successfully completed process motion plan for selected job");
  }
}

void CRSApplicationWidget::onApproveJob()
{
//  crs_application::TaskResponse res = app_->approveJob(boost::bind(&CRSApplicationWidget::onApproveJobComplete, this, _1));
//  if(!res.success)
//  {
//    // TODO: need to emit a signal here to go into a different state?

//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//  }
}

void CRSApplicationWidget::onApproveJobComplete(const bool approved)
{
  if(approved)
  {
    emit jobApproved();
    showWarningDialog("Trajectory Approved","Should now be able to execute");
    ui_->push_button_execute->setEnabled(true); // this should not need to be here, assignPropertiesExecutionReadyState sets true already
    ui_->tab_execution->setEnabled(true); // this should not need to be here, assignPropertiesExecutionReadyState sets true already
  }
  else
  {
    // TODO: need to emit a signal here to go into a different state?

    emit showWarningDialog("Trajectory Rejected", "The robot trajectories in this job have been rejected. Please attempt to re-plan with different parameters");
  }
}

void CRSApplicationWidget::onExecuteJob()
{
//  crs_application::TaskResponse job_log_res = app_->addJobLog();
//  if (job_log_res.success)
//  {
//    ROS_INFO_STREAM(job_log_res.message);
//  }
//  else
//  {
//    ROS_ERROR_STREAM(job_log_res.message);
//  }

//  crs_application::TaskResponse res = app_->executeJob(boost::bind(&CRSApplicationWidget::onExecuteJobComplete, this, _1));
//  if(!res.success)
//  {
//    QMessageBox::warning(this, "Application Error", QString::fromStdString(res.message));
//    emit planComplete();
//  }
}

void CRSApplicationWidget::onExecuteJobComplete(const bool success)
{
  if(!success)
  {
    emit showWarningDialog("Execution failure", "Failed to execute selected job");
    emit planComplete();
  }
  else
  {
    emit showInfoDialog("Execution Complete", "Successfully executed selected job");
    emit executionComplete();
  }
}

void CRSApplicationWidget::onShowWarningDialog(const QString& title,
                                                const QString& text)
{
  QMessageBox::warning(this, title, text);

  // Emit a signal telling anyone who is listening (namely the userControlCallback method) that the dialog window has been closed
  emit warningDialogClosed();
}

void CRSApplicationWidget::onShowInfoDialog(const QString& title,
                                             const QString& text)
{
  QMessageBox::information(this, title, text);
}

void CRSApplicationWidget::onCancelCurrentTask()
{
//  app_->cancelAllTasks();

  // Enter the first state of the process
  emit partSelected();

  QMessageBox::information(this, "Cancel Task", "Current task has been cancelled");
}

void CRSApplicationWidget::onUpdateProgressBar(const int value)
{
  progress_bar_->setValue(value);
}

void CRSApplicationWidget::onActivePartSet(const bool localized, const bool verified)
{
  // Go to the first state
  emit noPartSelected();
  emit partSelected();
//  job_selector_widget_->onPartSelect();
  if(localized)
  {
    // Emit the required signals to get the state machine to the "localized" state
    emit ui_->push_button_start_localization->released();
    emit partLocalized();

    if(verified)
    {
      // Emit the required signals to get the state machine to the "select job" state
      emit ui_->push_button_verify_localization->released();
      emit partLocalizationVerified();
      emit noJobSelected();
    }
  }
}

void CRSApplicationWidget::availableManipulatorsChangedManipulationWidget(QStringList manipulators)
{
//  manual_manipulation_widget_->setAvailableManipulators(manipulators);
}

void CRSApplicationWidget::availableTCPLinksChangedManipulationWidget(QStringList tcp_links)
{
//  manual_manipulation_widget_->setAvailableTCPLinks(tcp_links);
}

} // namespace crs_gui
