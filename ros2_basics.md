
# ROS 2
These are notes I made while following [ROS2 Basics in 5 Days (C++)](https://app.theconstructsim.com/Course/117/)

## Libraries
ROS client libraries allow nodes written in various programming languages to communicate. A core ROS client library (RCL) implements the standard functionality needed by various ROS APIs. This makes it easier to write language-specific client libraries. The ROS2 team currently maintains the following client libraries:

-   rclcpp = C++ client library
-   rclpy = Python client library

Additionally, other client libraries have been developed by the ROS community. You can check out the following article for more details:  [here](https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Client-Libraries.html).



# Packages
ROS2 uses  **packages**  to organize its programs. Think of a package as  **all the files that a specific ROS program contains**; all its CPP files, Python files, configuration files, compilation files, launch files, and parameter files. Also, organizing your ROS2 programs in packages makes sharing them with other developers/users much easier.

In ROS2, you can create two types of packages: CMake (C++) packages and Python packages. For this course, though, you will focus on the first ones.

Every CMake package will have the following structure of files and folders:

-   `launch`  folder: Contains launch files
-   `src`  folder: Contains source files (CPP, Python)
-   `CMakeLists.txt`: List of Cmake rules for compilation
-   `package.xml`: Package metadata and dependencies

### Create a package
```bash
ros2 pkg create <package_name> --build-type ament_cmake --dependencies <package_dependencies>
```
### List/Find a package
```bash
ros2 pkg list | grep <package_name>
```

# Building with Colcon
ROS2 uses **colcon** instead of **catkin**. [Documentation](https://colcon.readthedocs.io/en/released/index.html)
The environment source file is under `~/ros2_ws/install/setup.bash`
```bash
cd ~/ros2_ws/
colcon build
source install/setup.bash
```

### Build individual Packages
```bash
colcon build --packages-select <package_name>
```

### colcon_cd
You can also confirm that your package is there by using the [colcon_cd](https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes) command. Which is setup like this to be able to find all the packages created in your `ros_ws/src`. It offers a command to change to the directory a package specified by its name is in. To enable this feature you need to source the shell script provided by that package. The script is named `colcon_cd.sh`. For convenience you might want to source it in the user configuration, e.g. `~/.bashrc`:
```bash
cd ~
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_ws" >> ~/.bashrc
. .bashrc
```
Now, you can quickly navigate the workspace and its subdirectories with:
```
colcon_cd               	# cd to ~/ros2_ws/
colcon_cd <package_name>	# cd to ~/ros2_ws/src/<package_name>
```
# ROS2 Run
Run an executable
```
ros2 run <package_name> <executable_file>
```
# ROS2 launch 
#### Run using
```
ros2 launch <package_name> <launch_file>
```

Example: `teleop_keyboard`
```py
# Import Modules
from launch import LaunchDescription
from launch_ros.actions import Node

# Function to return LaunchDescription object
def generate_launch_description():				
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            emulate_tty=True),
    ])
```

Import some modules from the `launch` and `launch_ros` packages. Then, define a function that will return a `LaunchDescription` object. 

Within the  `LaunchDescription`  object, generate a node where you will provide the following parameters:

1.  `package='package_name'`  is the name of the package that contains the code of the ROS program to execute
2.  `executable='binary_executable_name'`  is the name of the binary executable file that you want to execute
3.  `output='type_of_output'`  is the channel where you will print the output of the program
4.  `emulate_tty=True|False`  allows launch files to output colored log messages, Green=DEBUG, White=INFO, Orange=Warning, and Red=ERROR|Fatal.

# Configuring CMakeLists.txt
C++ programs require you to configure them in `~/ros2_ws/src/<package_name>/CMakeLists.txt`. You must at least:
### 1. Generate executable for each `cpp` file
```
add_executable(executable_name src/executable.cpp)
```
### 2. Add all the ament target dependencies for each executable
```
ament_target_dependencies(executable_name dependency1 dependency2...)
```
### 3. Add the package's executables into the workspace's install folder  at `~/ros2_ws/install/<package_name>/lib`
```
install(TARGETS
   executable_name
   DESTINATION lib/${PROJECT_NAME}
 )
```
### 4. Install launch files from `~/ros2_ws/src/package/launch` into `~/ros2_ws/install/package/lib`
```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

# Example of Creating Package, Node, Launch File & Configuring CMakeLists.txt

Create and verify package 
```bash
colcon_cd
cd src
ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
colcon_cd
colcon build
ros2 pkg list | grep my_package
```

Create `simple.cpp`
```bash
colcon_cd my_package
cd src
touch simple.cpp
```
Add simple functionality
```cpp
// This program creates an endless loop that repeats itself 2 times per second (2Hz) 
// until somebody presses Ctrl + C in the Shell
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ObiWan");
  // We create a Rate object of 2Hz
  rclcpp::WallRate loop_rate(2);

  // Endless loop until Ctrl + C
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
    rclcpp::spin_some(node);
    // We sleep the needed time to maintain the Rate fixed above
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
```

Create launch directory and launch file
```bash
colcon_cd my_package
mkdir launch
cd launch
touch my_package_launch_file.launch.py
chmod +x my_package_launch_file.launch.py
```
Populate `my_package_launch_file.launch.py`
```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='simple_node',
            output='screen'),
    ])
```
Modify `CmakeLists.txt` - Add the following lines to your **CMakeLists.txt** file, above the final **ament_package()** line.
```bash
add_executable(simple_node src/simple.cpp)
ament_target_dependencies(simple_node rclcpp)

install(TARGETS
   simple_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```
Your final **CMakeLists.txt** file should look like this:
```bash
cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(simple_node src/simple.cpp)
ament_target_dependencies(simple_node rclcpp)

install(TARGETS
   simple_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```
Build the package and source
```bash
colcon_cd
colcon build --packages-select my_package
source ~/ros2_ws/install/setup.bash

# Can be combined into one line
colcon_cd && colcon build && source ~/ros2_ws/install/setup.bash
# Or run from an alias defined in .bashrc or .bash_aliases
alias cbs="colcon_cd && colcon build && source ~/ros2_ws/install/setup.bash"
cbs
```
Execute the launch file
```bash
ros2 launch my_package my_package_launch_file.launch.py
```
If successful, it should output repeatedly until `Ctrl+C` termination
```
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2023-02-05-21-29-55-923818-3_xterm-9156
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [simple_node-1]: process started with pid [9158]
[simple_node-1] [INFO] [1675632596.169638449] [ObiWan]: Help me Obi-Wan Kenobi, you're my only hope
[simple_node-1] [INFO] [1675632596.669775328] [ObiWan]: Help me Obi-Wan Kenobi, you're my only hope
[simple_node-1] [INFO] [1675632597.169779167] [ObiWan]: Help me Obi-Wan Kenobi, you're my only hope
[simple_node-1] [INFO] [1675632597.669778950] [ObiWan]: Help me Obi-Wan Kenobi, you're my only hope
[simple_node-1] [INFO] [1675632598.169772178] [ObiWan]: Help me Obi-Wan Kenobi, you're my only hope
[simple_node-1] [INFO] [1675632598.669873085] [ObiWan]: Help me Obi-Wan Kenobi, you're my only hope
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[simple_node-1] [INFO] [1675632598.947620096] [rclcpp]: signal_handler(signal_value=2)
[INFO] [simple_node-1]: process has finished cleanly [pid 9158]
```

# ROS2 Topics
| Command | Function |
|----|----|
| `ros2 topic list` | Lists all topics |
| `ros2 topic list \| grep topic_name`   | Lists ONLY topic_name   |
| `ros2 topic info /topic_name`   | View topic data   |
| `ros2 topic echo /topic_name`   | Print topic output   |
| `ros2 topic find std_msg/msg/String` | Lists available topics of type `std_msg/msg/String`   |
|    |    |


# ROS2 Nodes

Here is an example of a simple node that publishes an `Int32` message to the `/counter` topic

`simple_topic_publisher.cpp`
```cpp
// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Initiate a Node named 'simple_publisher'
  auto node = rclcpp::Node::make_shared("simple_publisher");
  // Create a Publisher object that will publish on the /counter topic, messages of the type Int32
  // The 10 indicates the size of the queue
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter", 10);
  // Create a variable named 'message' of type Int32
  auto message = std::make_shared<std_msgs::msg::Int32>();
  // Initialize the 'message' variable
  message->data = 0;
  // Set a publish rate of 2 Hz
  rclcpp::WallRate loop_rate(2);

  // Create a loop that will go until someone stops the program execution
  while (rclcpp::ok()) {
    // Publish the message within the 'message' variable
    publisher->publish(*message);
    // Increment the 'message' variable
    message->data++;
    rclcpp::spin_some(node);
    // Make sure the publish rate maintains at 2 Hz
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
```

The `simple_topic_publisher.cpp` script has been written using the  **old-school**  programming method. We say this because it is similar to the way you would have written this script in ROS1. In ROS2, though, this style of coding is going to become depreciated. You may be asking, why? It is because of  **Composition**.

In ROS2, as a notable difference from ROS1, the concept of  **Composition**  is introduced. This means that you can compose (execute) multiple nodes in a single process. You will learn more about  **node composition**  in the following units.

To use node composition, you must program your scripts in a more object-oriented way. So, the first script above,  **`simple_topic_publisher.cpp`**, will be unable to use node composition.

Below, you can look at a script that does the same thing, but is coded using a composable method, using classes.



```simple_topic_publisher_composable.cpp```
```cpp
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
using namespace std::chrono_literals;

//Define class, which inherits from rclcpp::Node class
class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher()		// Constructor
  : Node("simple_publisher"), count_(0)		// Initialize Node with name, and variable count_ with 0
  {
	// Initialize publisher_ object
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    // Initialize timer_ object and bind to callback function
    timer_ = this->create_wall_timer(500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  // Define timer and publisher_ as Shared Pointers to their objects
  rclcpp::TimerBase::SharedPtr timer_;								
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  // Declare counter variable
  size_t count_;
  // Define callback function
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create SimplePublisher object and make it spin until termination
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}
```


# ROS2 Messages/Interfaces

Topics handle information through messages (interfaces). There are different types of messages. In ROS1, you know this as **messages**. However, in ROS2, these messages are known as **interfaces**. Interfaces for topics are defined in  **.msg**  files, which are located inside a  **msg**  directory of a package.

Get information about an interface. 
```
ros2 interface show <message>
ros2 interface show std_msgs/msg/Int32
```
In this case, the **Int32** interface has only one variable of type **int32**, named **data**. This Int32 interface comes from the package std_msgs, which you can find in its **msg** directory.


# Example cmd_vel Publisher

Create `move_robot.cpp`
```bash
colcon_cd topic_publisher_pkg
cd src
touch move_robot.cpp
```
Create Node and publisher
```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("move_robot") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MoveRobot::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.1;
    message.angular.z = 0.1;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}
```

Create launch directory and launch file
```bash
colcon_cd topic_publisher_pkg
touch move_robot.launch.py
chmod +x move_robot.launch.py
```
Populate `move_robot.launch.py`
```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_publisher_pkg',
            executable='move_robot',
            output='screen'),
    ])
```
Modify `CmakeLists.txt` - Add the following lines to your **CMakeLists.txt** file.
```bash
find_package(geometry_msgs REQUIRED)

add_executable(move_robot src/move_robot.cpp)
ament_target_dependencies(move_robot rclcpp geometry_msgs)

install(TARGETS
   simple_topic_publisher
   simple_topic_publisher_composable
   move_robot
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```
The robot should start moving in a circle. After killing the program, the robot will continue following the most recent `cmd_vel` so to stop the robot, use:
```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
``` 

