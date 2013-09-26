baxter
======

Unofficial Baxter packages that add-on to the Rethink SDK. It is entirely written in C++ and currently contains Gazebo simulation and pick and place MoveIt code for Baxter. 

The groovy-devel branch is no longer being actively developed but on going development continues in the hydro-devel branch by [Dave Coleman](http://davetcoleman.com). 

![alt tag](https://raw.github.com/davetcoleman/baxter/hydro-devel/baxter_pick_place/resource/BaxterPickPlace.png)

**Note:** This is the ROS Groovy readme version. See hydro-devel branch for ROS Groovy instructions.

## Prequisites

 * A Baxter with dual parallel grippers, or the desire to see one in simulation
 * [ROS Groovy](http://wiki.ros.org/groovy/Installation)
 * Access to the private Rethink [sdk-examples](https://github.com/RethinkRobotics/sdk-examples) repository - we are using the baxter_interface and head_control packages from the SDK. Contact [Dave](davetcoleman@gmail.com) if you should have access to this.
 * Setup Github - the git@github.com urls, below, only work if you have [Setup Github](https://help.github.com/articles/set-up-git) and generated [SSH Keys for Github](https://help.github.com/articles/generating-ssh-keys).
 * Install wstool package
    ```
    sudo apt-get install python-wstool
    ```
 * Temporary [issue](https://github.com/ros/ros_comm/issues/283): install [ros_comm](https://github.com/ros/ros_comm) from source in a separate catkin workspace.

## Baxter Installation

* Create a catkin workspace and cd into it:

```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
```

* Checkout this repo

```
    git clone git@github.com:davetcoleman/baxter.git -b groovy-devel
```

* Also install from source a few customized repositories:

```
    git clone git@github.com:RethinkRobotics/sdk-examples.git -b gazebo_dev
    git clone git@github.com:davetcoleman/baxter_common.git -b dual_parallel_grippers
    git clone git@github.com:davetcoleman/block_grasp_generator.git -b groovy-devel
    git clone git@github.com:davetcoleman/ros_controllers -b velocity_position_controller
    git clone git@github.com:ros-simulation/gazebo_ros_pkgs.git
    git clone git@github.com:ros-controls/ros_control.git
    git clone git@github.com:ros-controls/control_toolbox.git
    git clone git@github.com:ros-controls/realtime_tools.git 
    
```

* Disable duplicate packages

    There is currently a duplication of packages in sdk-examples and baxter_common that must be fixed manually. This issue should be fixed in Rethink's next release of their SDK:

    ```
    touch sdk-examples/baxter_description/CATKIN_IGNORE
    touch sdk-examples/baxter_msgs/CATKIN_IGNORE
    ```

* Install dependencies

```
    cd ~/catkin_ws/
    rosdep install --from-paths . --ignore-src --rosdistro groovy -y
```

* Build

```
    catkin_make
```
You may need to run this command multiple times if there is a message dependency issue.

## Bringup Baxter

### Hardware

 * Turn on baxter
 * Enable robot:

    ```
    rosrun tools enable_robot.py -e
    ```

### Simulation 

 * Start simulation with controllers:
   ```
   roslaunch baxter_gazebo baxter_world.launch
   ```
   By default the position controllers are started. To switch, use the JointCommandMode topic as documented in the Baxter SDK.

 * Optional: Test/tune the velocity controllers or position controllers using a RQT dashboard GUI. Make sure you are in the right joint command mode when using these:

   ```
   roslaunch baxter_control baxter_sdk_position_rqt.launch
   ```
   or
   ```
   roslaunch baxter_control baxter_sdk_velocity_rqt.launch 
   ```

## Start MoveIt

Works with simulation or hardware:

 * Start MoveIt:

   ```
   roslaunch baxter_moveit_config baxter_bringup.launch
   ```

## Pick and place demo

   ```
   roslaunch baxter_pick_place baxter_pick_place.launch
   ```

## Develop and Contribute

See [Contribute](https://github.com/osrf/baxter/blob/master/CONTRIBUTING.md) page.
