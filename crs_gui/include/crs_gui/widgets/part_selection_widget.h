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

#ifndef CRS_GUI_WIDGETS_PART_SELECTION_WIDGET_H
#define CRS_GUI_WIDGETS_PART_SELECTION_WIDGET_H

#include <QWidget>

#include <string>

class QListWidgetItem;

namespace Ui
{
class PartSelection;
}

namespace crs_gui
{
class PartSelectionWidget : public QWidget
{
  Q_OBJECT
public:
  PartSelectionWidget(QWidget* parent = nullptr,
                      std::string database_directory = std::string(std::getenv("HOME")) + "/.local/share/"
                                                                                          "offline_generated_paths");
Q_SIGNALS:
  /** @brief Signal emitted when Load Selected Part is clicked with name of the part selected*/
  void partSelected(std::string);
  /** @brief Signal emmited when Load Selected Part is clickd. First arg is part selected. Second arg is path to
   * toolpath yaml */
  void partPathSelected(std::string, std::string);

protected Q_SLOTS:

  void refreshPartsList();
  void onPartSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  void onPartSelected();

private:
  Ui::PartSelection* ui_;

  std::string database_directory_;
};

}  // namespace crs_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H
