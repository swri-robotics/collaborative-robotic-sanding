# CRS_GUI
This package contains a series of widgets that make up the system gui. Each widget has a demo node that can be used for testing.

## Setup
This will likely change in the future, but current part selection relies on a specific directory structure. Set it up as follows
1) Create this directory `~/.local/share/offline_generated_paths`
2) Create a subdirectory per part. Example: `mkdir ~/.local/share/offline_generated_paths/part_1`
3) In each part subdirectory anything with a .yaml extension will be treated as a toolpath. Anything with a .ply extension will be treated as the mesh

## CRS Application widget
Main application widget that contains the other widgets. Launch demo with
```
ros2 launch crs_gui crs_application_demo.launch.xml
```

## Part Selection widget
This widget allows the user to select a part for processing. Launch the demo with
```
ros2 run crs_gui crs_gui_part_selection_demo
```

## Polygon Area Selection widget
This widget allows the user to select a region on the mesh. Launch the demo with
```
ros2 launch crs_gui area_selection_demo.launch.xml
```

## State Machine Interface Widget
This widget allows the user to navigate around the state machine. Launch the demo with
```
ros2 launch crs_gui state_machine_interface_demo.launch.xml
```
