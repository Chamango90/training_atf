```eval_rst
.. NOTE:: This instruction was tested on ``Ubuntu 16.04`` and ``ROS Kinetic Kame``.
```
# Prepare workspace

## Prerequisites
  * Install Ubuntut (16.04)
  * Install `ros-kinetic-dektop-full` as described [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Create a Catkin workspace
  ```bash
  source /opt/ros/kinetic/setup.bash
  mkdir -p ~/training_ws/src
  cd ~/training_ws
  catkin_init_workspace src
  catkin_make
  ```

  ```eval_rst
  .. WARNING:: To find packages for the new workspace you have to source every terminal: ``source ~/training_ws/devel/setup.bash``
  ```
  ```eval_rst
  .. TIP:: Add the command to your bash configuration: ``echo "source/opt/ros/kinetic/setup.bash" >> ~/.bashrc``
  ```

## Install ATF

  ```bash
  cd ~/training_ws/src
  git clone https://github.com/ipa-fmw/atf
  ```

## Install Turtlebot3

  Install the required navigation and Turtlebot3 packages:
  ```bash
  sudo apt-get install ros-kinetic-turtlebot3 ros-kinetic-turtlebot3-simulation
  ```

## Install the dependendies

  ```bash
  sudo rosdep init
  rosdep update
  cd ~/training_ws
  rosdep install --from-path src -i -y
  ```

## Compile sources

  ```bash
  cd ~/training_ws && catkin_make
  ```
