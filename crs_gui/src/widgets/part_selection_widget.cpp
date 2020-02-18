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

#include "crs_gui/widgets/part_selection_widget.h"

// TODO: Cull these includes
#include <map>
#include <regex>
#include <string>

#include <QFileDialog>
#include <QMessageBox>
#include <QTableView>
#include <QString>

#include <boost/filesystem.hpp>

#include "ui_part_selection.h"

namespace crs_gui
{
PartSelectionWidget::PartSelectionWidget(QWidget* parent, std::string database_directory)
  : QWidget(parent), ui_(new Ui::PartSelection), database_directory_(database_directory)
{
  ui_->setupUi(this);

  // Connect the signals and slots
  connect(ui_->push_button_refresh_parts_list, &QPushButton::clicked, this, &PartSelectionWidget::refreshPartsList);
  refreshPartsList();
  connect(ui_->list_widget_parts, &QListWidget::currentItemChanged, this, &PartSelectionWidget::onPartSelectionChanged);
  connect(ui_->push_button_load_selected_part, &QPushButton::clicked, this, &PartSelectionWidget::onPartSelected);
}

PartSelectionWidget::~PartSelectionWidget() = default;

void PartSelectionWidget::refreshPartsList()
{
  using namespace boost::filesystem;
  std::vector<path> part_dirs;
  for (directory_iterator itr(database_directory_); itr != directory_iterator(); itr++)
  {
    if (is_directory(itr->path()))
      part_dirs.push_back(itr->path());
  }

  // Retrieve part info from the database
  if (!part_dirs.size())
  {
    // If the function failed, create a warning pop-up box.
    std::string message = "Found no parts in " + database_directory_;
    QMessageBox::warning(this, "Database Communication Error", QString::fromStdString(message));
  }
  else
  {
    ui_->list_widget_parts->clear();
    for (auto& part : part_dirs)
    {
      // Gui display listing parts to user
      QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(part.stem().string()));
      item->setData(Qt::ItemDataRole::UserRole, QVariant(QString::fromStdString(part.stem().string())));
      ui_->list_widget_parts->addItem(item);
    }
  }
  return;
}

void PartSelectionWidget::onPartSelectionChanged(QListWidgetItem* current, QListWidgetItem*)
{
  // Change the description display based on which part is selected
  if (current != nullptr)
  {
    // Get selected part path
    using namespace boost::filesystem;
    path part(ui_->list_widget_parts->currentItem()->data(Qt::ItemDataRole::UserRole).toString().toUtf8());
    path part_paths_dir(database_directory_);
    part_paths_dir += "/" + part.string();

    // Get all yaml files in that directory
    std::vector<path> part_paths;
    for (directory_iterator itr(part_paths_dir); itr != directory_iterator(); itr++)
    {
      int t = 2;
      if (itr->path().extension() == ".yaml")
        part_paths.push_back(itr->path());
    }

    // Display all yaml files
    ui_->list_widget_part_paths->clear();
    for (auto& paths : part_paths)
    {
      // Gui display listing parts to user
      QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(paths.stem().string()));
      item->setData(Qt::ItemDataRole::UserRole, QVariant(QString::fromStdString(paths.stem().string())));
      ui_->list_widget_part_paths->addItem(item);
    }
  }
}

void PartSelectionWidget::onPartSelected()
{
  if (ui_->list_widget_parts && ui_->list_widget_parts->currentItem() && ui_->list_widget_part_paths &&
      ui_->list_widget_part_paths->currentItem())
  {
    std::string current_part =
        ui_->list_widget_parts->currentItem()->data(Qt::ItemDataRole::UserRole).toString().toUtf8().constData();
    std::string current_path =
        ui_->list_widget_part_paths->currentItem()->data(Qt::ItemDataRole::UserRole).toString().toUtf8().constData();
    emit partSelected(current_part);
    emit partPathSelected(current_part, current_path);
  }
  else if (ui_->list_widget_parts && ui_->list_widget_parts->currentItem())
  {
    std::string current_part =
        ui_->list_widget_parts->currentItem()->data(Qt::ItemDataRole::UserRole).toString().toUtf8().constData();
    emit partSelected(current_part);
  }
}

}  // namespace crs_gui
