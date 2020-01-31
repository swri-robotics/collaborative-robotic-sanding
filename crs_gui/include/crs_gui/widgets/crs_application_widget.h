/*
 * Copyright 2020 Southwest Research Institute
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

#ifndef CRS_GUI_WIDGETS_CRS_APPLICATION_WIDGET_H
#define CRS_GUI_WIDGETS_CRS_APPLICATION_WIDGET_H

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>

//#include <geometry_msgs/PoseStamped.h>
//#include <ros/node_handle.h>
//#include <ros/service_client.h>

//#include <crs_msgs/AllowUserControlAction.h>
//#include <actionlib/server/simple_action_server.h>

//#include <crs_gui/widgets/manual_manipulation_widget.h>


class QPushButton;
class QStateMachine;
class QProgressBar;
class QProgressDialog;

namespace Ui
{
class CRSApplication;
}

//namespace crs_application
//{
//class ApplicationContextBase;
//typedef typename std::shared_ptr<ApplicationContextBase> ApplicationContextBasePtr;
//}

namespace crs_gui
{
class PartSelectionWidget;
//class JobSelectorWidget;
//class TrajectoryApprovalWidget;
//class DatabaseLogWidget;
 
class CRSApplicationWidget : public QWidget
{
Q_OBJECT
public:

  CRSApplicationWidget(rclcpp::Node::SharedPtr node,
                       QWidget* parent = nullptr,
                       std::string database_directory = std::string(std::getenv("HOME")) + "/.local/share/offline_generated_paths");

Q_SIGNALS:

  void showWarningDialog(const QString& title,
                         const QString& text);

  void warningDialogClosed();

  void showInfoDialog(const QString& title,
                      const QString& text);

  // Signals to connect to RVIZ panel/display
  void showManipulationWidget();
  void hideManipulationWidget();

  void manipulatorChangedManipulationWidget(QString manipulator);
  void tcpLinkChangedManipulationWidget(QString tcp_link);

  void showCartesianManipulationWidget();
  void hideCartesianManipulationWidget();

  void showJointManipulationWidget();
  void hideJointManipulationWidget();

  void resetToCurrentStateManipulationWidget();

  void showTrajectory();
  void hideTrajectory();

  // State machine signals
  void partLocalized();
  void partLocalizationVerified();
  void noPartSelected();
  void partSelected();
  void noJobSelected();
  void jobSelected();
  void planComplete();
  void jobApproved();
  void executionComplete();
  void updateProgressBar(const int);

public Q_SLOTS:
  void availableManipulatorsChangedManipulationWidget(QStringList manipulators);
  void availableTCPLinksChangedManipulationWidget(QStringList tcp_links);

protected Q_SLOTS:

  void onPartSelected(const std::string);

  void onPartSelectionStatusChanged(const bool status);

  virtual void onPartRemoved(const bool success);

  void onSafelyGoHome();

  void onScanBooth();

  void onAlignToScan();

  void onStartLocalization();

  void onStartLocalizationVerification();

  void onLoadLocLog();
  
  void onDetailedScanLoc();

  virtual void onPlanJob();

  virtual void onApproveJob();

  virtual void onExecuteJob();

  void onShowWarningDialog(const QString& title,
                           const QString& text);

  void onShowInfoDialog(const QString& title,
                        const QString& text);

  void onCancelCurrentTask();

  void onUpdateProgressBar(const int value);

protected:

  /**
   * @brief Creates a state machine to manage the properties of the UI (i.e. enabled state, color, etc.).
   * This state machine is a "nested sequential" type state machine, meaning that states are
   * connected sequentially by "forward" transitions, and any given state is connected by "reverse"
   * transitions to all previous states
   */
  void createStateMachine();

//  void userControlCallback(const crs_msgs::AllowUserControlGoalConstPtr& goal);

  /* Notification callbacks
   *
   * Note: these methods are given as arguments to the application class which eventually calls
   * them from a ROS thread from within an action callback. Therefore we cannot create a QMessageBox
   * directly inside them (since this function is not being called by the Qt GUI thread);
   * thus we must emit a Qt signal to trigger the creation of the dialog window instead.
   *
   */
  void onSafelyWentHome(const bool success);
  void onScanBoothComplete(const bool success);
  void onLocalizationFeedback(const int pct_complete);
  void onLocalizationComplete(const bool success);
  void onLocalizationVerificationFeedback(const int pct_complete);
  void onLocalizationVerificationComplete(const bool success);
  void onJobSelectionStatusChanged(const bool status);
  void onPlanJobComplete(const bool success);
  void onApproveJobComplete(const bool approved);
  void onExecuteJobComplete(const bool success);
  void onActivePartSet(const bool localized, const bool verified);

  Ui::CRSApplication* ui_;

//  QStateMachine* sm_;

  QProgressBar* progress_bar_;
  QProgressDialog* progress_dialog_;

  rclcpp::Node::SharedPtr node_;

//  actionlib::SimpleActionServer<crs_msgs::AllowUserControlAction> user_control_action_server_;

//  crs_application::ApplicationContextBasePtr app_;

  PartSelectionWidget* part_selector_widget_;
//  JobSelectorWidget* job_selector_widget_;
//  TrajectoryApprovalWidget* trajectory_approval_widget_;
//  ManualManipulationWidget* manual_manipulation_widget_;
//  DatabaseLogWidget* database_log_widget_;
};

} // namespace crs_gui

#endif // crs_GUI_WIDGETS_APPLICATION_WIDGET_BASE_H
