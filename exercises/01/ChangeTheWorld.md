---
title: "Exercise 1 - Change the World"
documentclass: scrreprt
subtitle: "183.660 Mobile Robotics"
numbersections: true
date: \today
---

# Introduction {-}

The purpose of this exercise is to get you familiar with ROS and to prepare your system for the upcoming assignments. It is divided into three parts:

1. Simulation (40 Points)
2. ROS2 navigation2 (50 Points)
3. Documentation (10 Points)

Read all instructions carefully and **include all commands used as part of the documentation!** Further, document it in such detail, that your actions can be reproduced by others.

Lastly, prepare your system according to the instructions of the [project root](https://gitlab.tuwien.ac.at/lva-mr/2024/project).

# Simulation (40 Points)

## tmux (6 Points)

Throughout these exercises, the tool __tmux__ will be used extensively. Therefore, we want you to get familiar with it and it's shortcuts.

Enable mouse support for _tmux_. Memorize and document five important short-cuts. Tell us how you enabled the mouse integration and how you can use the middle button for copy and paste, how to split the screen vertically or horizontally and how to detach and attach to sessions. We would also like you to get familiar with windows/tabs. Check out how new windows are created and renamed. 

+ Enable mouse support: 1 Point
+ 5 shortcuts: 1 Point
+ Using middle mouse button: 1 Point
+ Split screen: 1 Point
+ Attach/Detach: 1 Point
+ Handling Windows: 1 Point

## `teleop_twist_keyboard` (4 Points)

Use the `teleop_twist_keyboard` node to drive a simulated robot. Document your actions.

## Stage (30 Points)

In the first part you have to get to know how the simulation, called Stage, works and how to change it. Make a copy of the `cave.world` file and change it in such a way, that it looks like your favorite environment. For example, I created a Bender environment. Your documentation must contain a screenshot of your environment and in addition, you have to tell us in a few sentences (2-3) how you did it. The world file nearly explains itself, but if you like to know more, have a look at: [http://rtv.github.io/Stage/]([https://](http://rtv.github.io/Stage/)) (be aware that there is an old version of Stage on SourceForge).

![Bender 1](./res/stage_bender01.png){ width=50% }
![Bender 2](./res/stage_bender02.png){ width=50% }

# Nav2 (50 Points)

[nav2](https://docs.nav2.org/) includes nodes to

* Create a map using a SLAM approach (`slam_toolbox`),
* Save the map (`map_saver`),
* Publish a map (`map_server`),
* Self localize (`amcl`),
* Navigate (`move_base`).

Your task is to use these packages to navigate between two points and to document your approach. Screenshots and 2 sentences are normally enough, but you might have to explain what you did in your submission talk.

## Prerequisites {-}

You have to install all relevant tools, if they have not yet been installed.

```sh
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-teleop-tools \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-mouse-teleop \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup
```

### Update the mobile robotics project root

```sh
cd $MR_DIR
git pull
cd ws02/src/mr
git pull
```

## RViz (10 Points)

Start the simulation and `teleop_twist_keyboard`, same as in the previous sections. In this task you need to visualize the environment of the robot using `rviz`.

```sh
ros2 run rviz2 rviz2 
```
- Define __base_link__ as *Fixed Frame* and add the LaserScan (topic: __base_scan__) as well as the TF topic. (4 Points)
- When inspecting the settings rviz presents for displayed topics, you can find options regarding the topics Quality of Service. Document what this term means in the context of ros2. (2 Points)
    - Which QoS-profiles are available in ros2 and how do they differ from one another? (2 Points)
    - To which degree are different QoS settings compatible with eachother? (2 Points)

![rviz laser view](./res/rviz_view_laser.png){width=60%}

## slam_toolbox (10 Points)

Create a map of your own environment using the `slam_toolbox`. Visit [https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html]([https://](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)) to find the correct command to start mapping. To see the result, you have to add the map topic to your RViz view and to change the Fixed Frame to `map`. 

![After mapping](./res/rviz_view_gmapping.png)

__Hints:__ 

- It helps to use *tmux* instead of multiple terminal windows or tabs. You need at least five terminals!
 - `ros2 launch stage_ros2 stage.launch.py`
 - `ros2 run rviz2 rviz2`
 - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
 - `your command for mapping`
 - `to save the map (next section)`
- Checkout the readme.md to the mr_nav project in your workspace! There is a working configuration!!

## `map_saver` (5 Points)

Save your created map using the `map_server` node and the command documented at [https://index.ros.org/p/nav2_map_server/](https://index.ros.org/p/nav2_map_server/). Do not overwrite the cave map! Make your own map!

## `map_server` (5 Points)

Fix any holes or other issues present in your map. Then, use the `map_server` to publish the map statically to the `/map` topic. (Make sure not to publish to a topic from multiple sources!)

__Hints:__ 
 
- <https://answers.ros.org/question/406036/how-to-publish-map-in-ros2-galactic/>
- Pay attention to the indentations in the configuration file.
- Do not forget to send the relevant lifecycle messages from a separate console

## AMCL Self Localization (10 Points)

AMCL should be used for self-localization to establish a link between your map and the robot base. Details can be found at [https://github.com/ros-planning/navigation2/tree/main/nav2_amcl](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl). (Sadly, the documentation is still lacking and now very good.)

To start the self-localization, run the below command. After this, you need to initialize the system, by specifying an initial pose using the "2D Pose Estimate" button in rviz.

```sh
ros2 launch mr_nav localization_launch.py map:=$MR_DIR/.......
```

Now you can drive using the teleop node, and you will hopefully see the robot on the correct spot. The particles used for the self localization can be visualized as a `PoseArray` from the topic ParticleCloud.
You will receive 9 Points for the working amcl and 1 point if you can drive with `mouse_teleop` as shown in the screenshot.

![Mouse Teleop, AMCL, Rviz](./res/amcl_mouse_teleop.png)

## Nav2 (5 Points)

The navigation package is a very complex, therefore will provide you with a working launch file.

```sh
ros2 launch mr_nav navigation_launch.py
```

After you launched move_base, you should be able to tell the robot where it should go (1 Point) using the 2D Nav Goal button in RViz.
To get some indication of what is going on, you can view the cost maps (2 Points) as well as the path (2 Points).

![AMCL Nav2](./res/amcl_nav2_a.png)

![AMCL Nav2](./res/amcl_nav2_b.png)

## Tweaking Navigation (2 Points)

Check out the navigation stack https://docs.nav2.org/, and you will see how complex it is. A good way to tweak it is to have a look at the navigation_launch.py and the parameter files used there. Change some parameters (e.g. a costmap parameter) and document the differences.

## Launch File (3 Points)

Create a ros2 python launch file to start everything except the simulation environment (hint: take a look at the already existing launch files in the mr_nav/launch/nav2-folder).

# Documentation (10 Points)

* Your documentation should not be more than 5-6 pages (1 Point) - primarily screenshots (Alt-Print or Shift Print). In this assignment, it will be the only thing you have to submit, no code.
* Document every step with screenshots, for the tasks "Tweaking Navigation" and "Launch File" you can add the files with a brief inline documentation (7 Points).
* The first page includes your full name and student ID, and a table showing how many points you reached. (2 Points)

# Optional exercises and bonus points​ (5 Points)

## Optional exercises

Work through the [Using turtlesim and rqt](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

## Bonus points

Document the following tasks in your Documentation to get points.

* Use `rqt_graph` to visualize the nodes running (1 Point) 
* Use `rqt_plot` to plot the history of selected forward velocities and rotational velocities (1 Point) 
* Document the pushing frequents of the *twist* message on *cmd_vel* using `ros2 topic` (1 Point) 
* Use `ros2 topic pub` to publish a number or string (1 Point) 
* Use `ros2 topic echo` to show what you have published with `ros2 topic pub` (1 Point) 

