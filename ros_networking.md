# Setting Up ROS Network On Multiple Machines

Browse the ROS Wiki for reference:
http://wiki.ros.org/ROS/NetworkSetup

http://wiki.ros.org/ROS/Tutorials/MultipleMachines

http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2FROS_HOSTNAME

Example setup: a remote robot computer running `roscore`, and I'm on my laptop connecting to the robot.
|System| Type | IP | Hostname |
|--|--|--|--|
| Robot | MASTER | 192.168.0.33 | robotHostname |
| Laptop | Client | 192.168.0.69 | laptopHostname |

> Don't forget to replace the dummy IP and hostnames with your own lolol

# 1. On `robot`

* In `/etc/hosts`, add hostname `laptopHostname` and its IP

`/etc/hosts`
```bash
127.0.0.1       localhost
127.0.1.1       robotHostname

192.168.0.69    laptopHostname
```

* In `.bashrc`,  set `ROS_HOSTNAME` and `ROS_MASTER_URI`

`~/.bashrc`
```bash
export ROS_HOSTNAME=robotHostname
export ROS_MASTER_URI=http://robotHostname:11311

alias printROSEnv="env | grep ROS_IP && env | grep ROS_MASTER_URI && env | grep ROS_HOSTNAME"
printROSEnv
```
* Opening a new terminal or `source ~/.bashrc` should output:
```
ROS_IP=192.168.0.33                         # Robot's IP
ROS_MASTER_URI=http://robotHostname:11311   # Using Robot's Hostname
ROS_HOSTNAME=robotHostname                  # Robot's Hostname
```
* Now running `roscore` from a terminal on `robot` should give something like
```bash
started roslaunch server http://robotHostname:34139/
ros_comm version 1.14.11

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.11

NODES

auto-starting new master
process[master]: started with pid [25476]
ROS_MASTER_URI=http://robotHostname:11311/
```
# 2. On `laptop`

* In `/etc/hosts`, add hostname  `robotHostname` and its IP

`/etc/hosts`
```bash
127.0.0.1       localhost
127.0.1.1       laptopHostname

192.168.0.33    robotHostname
```
* In `.bashrc`,  set `ROS_IP` to the laptop's IP, `ROS_HOSTNAME` to the laptop's hostname and `ROS_MASTER_URI` using the robot's hostname

`~/.bashrc`
```bash
export ROS_IP=192.168.0.69                        # Laptop's IP
export ROS_MASTER_URI=http://robotHostname:11311  # Using robot's hostname
export ROS_HOSTNAME=laptopHostname                # Laptop's Hostname

alias printROSEnv="env | grep ROS_IP && env | grep ROS_MASTER_URI && env | grep ROS_HOSTNAME"
printROSEnv

# For convenience, you can make an alias to switch between local and remote masters
alias robotMaster="export ROS_IP=192.168.0.69; export ROS_MASTER_URI=http://robotHostname:11311 ; export ROS_HOSTNAME=laptopHostname"
alias localMaster="export ROS_MASTER_URI=http://localhost:11311 ; export ROS_HOSTNAME=localhost"
```

* Opening a new terminal should output:
```
ROS_IP=192.168.0.69
ROS_MASTER_URI=http://robotHostname:11311
ROS_HOSTNAME=laptopHostname
```

---

### Now you should be good to go!

If not, double check if you replaced the dummy IP and hostnames with your own lolol
