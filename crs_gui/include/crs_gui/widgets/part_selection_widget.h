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
#include <QString>
#include <memory>
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
  ~PartSelectionWidget();
Q_SIGNALS:
  /** @brief Signal eitted when Load Selected Part is clicked with name of the part selected*/
  void partSelected(QString);
  /** @brief Signal emmited when Load Selected Part is clicked. First arg is part selected. Second arg is path to
   * toolpath yaml */
  void partPathSelected(QString, QString);

protected Q_SLOTS:

  /** @brief Checks for new parts and updates the list */
  void refreshPartsList();
  /** @brief Checks the toolpaths for the selected part and populates the display*/
  void onPartSelectionChanged(QListWidgetItem* current, QListWidgetItem* previous);
  /** @brief Gets part name of selection and triggers partSelected and partSelectedPath signals*/
  void onPartSelected();

private:
  std::unique_ptr<Ui::PartSelection> ui_;

  /** @brief Directory where the part directories are located */
  std::string database_directory_;
};

}  // namespace crs_gui

#endif  // OPP_GUI_WIDGETS_TOOL_PATH_PLANNER_WIDGET_H
