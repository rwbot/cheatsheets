# Configuring VSCode for ROS & C++
## Download Plugins
Install the following extensions as detailed [here](https://erdalpekel.de/?p=157)
1. [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
2. [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake)
3. [ROS](https://marketplace.visualstudio.com/items?itemName=ajshort.ros)
4. [Clang Format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)
`bash
sudo apt-get install clang-format
`
Also, go to the settings by typing ‘Ctrl + ,’ and search for ‘editor.formatOnSave’ and set it to true. Otherwise the files need to be formatted with manual commands.
5. [vscode-icons](https://marketplace.visualstudio.com/items?itemName=robertohuertasm.vscode-icons)

## Setup Intellisense
1. If you haven't already, navigate to **File > Open Folder** and open the top-level catkin workspace or a ROS package.
2. Navigate to **File > Save Workspace As...** and save (this is just to generate the `.vscode` folder and can be removed later).
3.  Expand the `.vscode` folder and open `c_cpp_properties.json` and paste the following **after** the `browse` element, but **before** the `includePath` element. Be wary of missing commas.
```json
"defines": [],
"compilerPath": "/usr/bin/gcc",
"cStandard": "gnu17",
"cppStandard": "c++14",
"intelliSenseMode": "gcc-x64",
```
4.  The file should look similar to this (except auto-populated with the packages in your workspace)
 ###  `c_cpp_properties.json`
```json
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "",
        "limitSymbolsToIncludedHeaders": false
      },
      "name": "ROS",
      "defines": [],
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu17",
      "cppStandard": "c++14",
      "intelliSenseMode": "gcc-x64",
      "includePath": [
        "~/pcl_ws/devel/include/**",
        "/opt/ros/melodic/include/**",
        "/opt/ros/melodic/include/**/**",
        "/opt/ros/melodic/share/fetch_gazebo/include/**",
        "/opt/ros/melodic/share/julius/include/**",
        "~/catkin_ws/src/pkg2/include/**",
        "~/catkin_ws/src/pkg1/include/**",
        "/usr/include/**"/**
      ]
    }
  ],
  "version": 4
}
```

## Configure Build, Clean and Run Tasks
1. Press **Ctrl + N** to open a new file, paste the code below. Then **Ctrl + S** to save the file, naming it **`tasks.json`**.  Be sure to save it in the same `.vscode` folder alongside the previous `c_cpp_properties.json`
 ###  `tasks.json`

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "type": "catkin",
            "args": [
                "build",
                "--force-cmake",
                "--continue"
            ],
            "problemMatcher": [
                "$catkin-gcc"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "label": "catkin: build all",
            "options": {
                "cwd": "${fileDirname}"
            }
        },
        {
            "label": "example_launch",
            "type": "shell",
            "command": "roslaunch example_pkg example.launch",
            "problemMatcher": [],
            "group": {
                "kind": "test",
                "isDefault": true
            }
        },
        {
            "label": "catkin clean",
            "type": "shell",
            "command": "catkin clean --all-profiles"
        }
    ]
}
```

2. Your `.vscode` folder should now look similar to this 
```
.vscode/
├── c_cpp_properties.json
├── tasks.json
└── workspace.code-workspace
```
3. Confirm the tasks were specified by using **Ctrl + Shift + B**  to run the `catkin build` task that was just made 




---
Ref:
[https://samarth-robo.github.io/blog/2020/12/03/vscode_ros.html](https://samarth-robo.github.io/blog/2020/12/03/vscode_ros.html)
[https://github.com/ms-iot/vscode-ros/issues/114#issuecomment-756783093](https://github.com/ms-iot/vscode-ros/issues/114#issuecomment-756783093)
