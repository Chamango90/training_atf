```eval_rst
.. NOTE:: This instruction was tested on ``Ubuntu 16.04`` and ``ROS Kinetic Kame``.
```
# Create ATF package

The current work-flow intend the developer to have a dedicated ROS package for the examined application.

## Download ATF template


  ```bash
  cd ~/training_ws/src
  git clone https://github.com/ipa-jfh/training_atf
  ```


## Launch file

There is one central application launch file which will be started by ATF.
It should contain all the rosnodes that are required to perform the test run. In this example we will need to run the Simulation and Navigation packages for Turtlebot3.
Besides, it must provide certain arguments which enable to start the test with the desired variations.

The following code represents the full launch script. Add it into the according `launch/application.launch` file of your ATF test package.

```eval_rst
.. literalinclude:: ./code/application.launch
   :language: xml
   :linenos:
```
The next step is to create the actual test application node, which ATF will automatically include into the launch file.

## Test application

This script contains the runtime of the test application and it will be automatically executed by ATF for every test case. If this program is finished it also terminates all the nodes that have been started and proceed to the next test. The general content of the file is the following. It creates an rosnode, containing the *Application* which is structured by an *init* and an *execute* function. These should be extended by the user to fit the needs of the test application. It provides two ways to run the node. Without any arguments it will perform a *ROS test* which will jump into the *Test* class (this should not be modified by the user). However, by using the *standalone* argument one could start it manually as a typical rosnode for debugging purpose.

```eval_rst
.. literalinclude:: ./code/application.py
   :language: python
   :linenos:
```

Add the following code after the existing imports (*l.8*) of `application.py`. They import the further necessary libraries regarding the ROS navigation and transformation stack.
Additionally, they cover two new functions that simply help to send a goal message to the ROS *MoveBaseAction*. The `move_base.wait_for_result()` line makes sure that the thread will wait for the action, in this case the movement, has finished.


```eval_rst
.. code-block:: python

  import actionlib
  from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
  from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
  from actionlib_msgs.msg import GoalStatus
  from tf.transformations import quaternion_from_euler

  def create_nav_goal(goal_2d):
      "Create a MoveBaseGoal with x, y (in m) and yaw (in rad)."
      rospy.loginfo(type(goal_2d))
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = '/map'
      goal.target_pose.pose.position.x = goal_2d[0]
      goal.target_pose.pose.position.y = goal_2d[1]
      q = quaternion_from_euler(0.0, 0.0, goal_2d[2])
      goal.target_pose.pose.orientation = Quaternion(*q.tolist())
      return goal

  def move_to_goal(move_base, goal):
      "Send a MoveBaseGoal to the MoveBaseAction."
      rospy.loginfo("Send goal to %s", goal.target_pose.pose)
      move_base.send_goal(goal)
      rospy.loginfo("Waiting for result...")
      move_base.wait_for_result()
      if move_base.get_state() == GoalStatus.SUCCEEDED: rospy.loginfo("Reached goal!")
      else: rospy.loginfo("Goal not reached!")
```

The next step is to structure the program and the evaluation. One can define certain sequences as *ATF test blocks* that can be examined regarding desired key performance indicators.
Therefore, it has to be embraced with *start* and a *stop* function call. In the provided test scenario we use only one block for the navigation task.
Finally, there is one more function which triggers the shutdown.

```eval_rst
.. TIP:: It is also possible to have multiple *ATF test blocks*, even nested ones.
```

Add the following lines to the end of the *init* function (*l.12*) to initialize the action client:

```eval_rst
.. code-block:: python

  self.move_base = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
  rospy.loginfo("Connecting to /move_base action")
  self.move_base.wait_for_server()
```

Replace the content of the *execute* function (*l.14-26*) with the following lines:

```eval_rst
.. code-block:: python

  # Insert your ATF test blocks here
  self.atf.start("move_to_goal")
  move_to_goal(self.move_base, create_nav_goal(rospy.get_param("/test_goal")))
  self.atf.stop("move_to_goal")
  self.atf.shutdown()
```


## Configurations

ROS application typically consists of a multitude of components and application variations. ATF assists the developer to test all variations automatically.
The ATF tool distinguishes therefore between three different sources:  
- robots            
- robot_envs    
- test_configs      

One example is to evaluate different type of robots. One can also examine different robot configurations of the same model e.g. by tuning parameters of the motion configuration. Moreover, there is the possibility to simulate the robot in different worlds, which leads to a set of test environments. One more set of variation is related to the examined performance indicators, which is denoted as test_configs.

With `config/test_suites.yaml` the user can specify which constellations of variations are of interest and generate a matrix of test suites.

The number of test will be: n<sub>tests</sub> = n<sub>robots</sub> x n<sub>robot_envs</sub> x n<sub>test_configs</sub>

Change its content for this example to the following:

```eval_rst
.. literalinclude:: ./code/test_suites.yaml
   :language: yaml
```

Further configurations of the ATF package can be done in the `config/test_generation_config.yaml` file. As what concerns this training one does not need to change anything. There is e.g. the option to define a certain number of repetitions, in order to eliminate stochastic uncertainty.

```eval_rst
.. literalinclude:: ./code/test_generation_config.yaml
   :language: yaml
   :linenos:
```


### Robots

For this training we want to use the *Turtlebot3* robot. It comes with two variations, one is called *Waffle* and one *Burger*. In order to run the tests with both robots we need to add the following files into the `config/robots` folder.

- `burger.yaml`
```eval_rst
.. literalinclude:: ./code/robots/burger.yaml
   :language: yaml
```
- `waffle.yaml`
```eval_rst
.. literalinclude:: ./code/robots/waffle.yaml
   :language: yaml
```

### Robot environments

We only want to test the application in one environment. Thus, add the following files into the `config/robot_envs` folder.

- `env1.yaml`
```eval_rst
.. literalinclude:: ./code/robot_envs/env1.yaml
   :language: yaml
```

### Tests

This section is dedicated to define the desired key performance indicators. As such, there are common metrics available to measure e.g. duration, distance and frequency. Further individual ones can be created by the developer.
In the navigation use-case, we want to use ATF to benchmark the application regarding the execution duration and the traveled path length.

We will also use this configuration to preset our desired goal poses for the navigation task.

```eval_rst
.. NOTE:: Alternatively, one could also set desired goal poses as arguments one each in a file of the *robot_envs* folder. The reason to define them in the test folder is that we want to have one dedicated groundtruth value per goal request and don't want to cross-test them.

.. NOTE:: The ground truth value is not required for benchmarking, but for the case you want to do CI (Continuous Integration).
```

Thus, the following files have to be added into the `config/test_configs` folder.

- `test1.yaml`
```eval_rst
.. literalinclude:: ./code/test_configs/test1.yaml
   :language: yaml
```
- `test2.yaml`
```eval_rst
.. literalinclude:: ./code/test_configs/test2.yaml
   :language: yaml
```

```eval_rst
.. NOTE:: Be aware that package requires a dependency to *atf_core* and in the `CMakeList.txt` the function call to *atf_test()*

```
