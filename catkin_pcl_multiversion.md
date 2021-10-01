## PCL_UNDERLAY

### Debug

```
cd ~/dbg_source_pcl_ws
catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin build 
```

### Release

```
cd ~/source_pcl_ws
catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build 
```

## PCL_WS - Separate Build Type Spaces

### Config
```
cd ~/pcl_ws

# Release Profile
source ~/source_pcl_ws/install/setup.bash
catkin config --profile release -x _rel --extend ~/source_pcl_ws/install  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Debug Profile
source ~/dbg_source_pcl_ws/install/setup.bash
catkin config --profile debug -x _debug --extend ~/dbg_source_pcl_ws/install  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

catkin profile set release
catkin profile remove default

```

### Build Release 
```
cd ~/pcl_ws
source ~/source_pcl_ws/install/setup.bash
catkin profile set release
catkin build --profile release
source devel_rel/setup.bash
```

### Build Debug
```
cd ~/pcl_ws
source ~/dbg_source_pcl_ws/install/setup.bash
catkin profile set debug
catkin build --profile debug
source devel_debug/setup.bash
```


### Alias
```
alias swspr="cd ~/pcl_ws; source devel_rel/setup.bash"
alias swspd="cd ~/pcl_ws; source devel_debug/setup.bash"

alias ccypr="cd ~/pcl_ws; catkin clean -y --profile release"
alias ccypd="cd ~/pcl_ws; catkin clean -y --profile debug"

alias cbptr="cd ~/pcl_ws && source ~/source_pcl_ws/install/setup.bash && catkin profile set release && catkin build --profile release && swspr"
alias cbptd="cd ~/pcl_ws && source ~/dbg_source_pcl_ws/install/setup.bash && catkin profile set debug && catkin build --profile debug && swspd"
```

## PCL_WS - Default Build Type Spaces

### Config
```
cd ~/pcl_ws

# Release Profile
source ~/source_pcl_ws/install/setup.bash
catkin config --profile release --extend ~/source_pcl_ws/install  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Debug Profile
source ~/dbg_source_pcl_ws/install/setup.bash
catkin config --profile debug --extend ~/dbg_source_pcl_ws/install  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

catkin profile set release
catkin profile remove default

```

### Build Release 
```
cd ~/pcl_ws
source ~/source_pcl_ws/install/setup.bash
catkin clean -y
catkin profile set release
catkin build --profile release
source devel/setup.bash
```

### Build Debug
```
cd ~/pcl_ws
source ~/dbg_source_pcl_ws/install/setup.bash
catkin clean -y
catkin profile set debug
catkin build --profile debug
source devel/setup.bash
```


### Alias
```
alias cbptr="cd ~/pcl_ws && source ~/source_pcl_ws/install/setup.bash && ccy && catkin profile set release && catkin build --profile release && sws"
alias cbptd="cd ~/pcl_ws && source ~/dbg_source_pcl_ws/install/setup.bash && ccy && catkin profile set debug && catkin build --profile debug && sws"
```
