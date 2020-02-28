# Project 3, Where Am I?
## Udacity Robotics Software Engineer Nanodegree
Where Am I? project of the Robotics Software Engineer Nanodegree program from Udacity.

## How to Launch the simulation?

#### Clone the package in catkin_ws/src/
```sh
$ cd /home/<project folder>/
$ git clone https://github.com/willhunt/robo_nd_project3.git robo_nd_project3
```

#### Build the `robo_nd_project3` package
```sh
$ cd /home/<project folder>/ 
$ catkin_make
```

#### Make sure to check and install any missing dependencies
```sh
$ rosdep install --from-paths src --ignore-src -r -y
```

#### Create map of world (if changed)
Run map creator with world file
```sh
$ gzserver src/pgm_map_creator/world/home_flattened.world
```
In another terminal
```sh
$ roslaunch pgm_map_creator request_publisher.launch
```

#### After building the package, source your environment
```sh
$ cd /home/<project folder>/
$ source devel/setup.bash
```

#### Once the `robo_nd_project3` package has been built, you can launch the simulation environment using:
```sh
$ roslaunch my_robot world.launch
```

#### In a new terminal launch the ball chaser node:
```sh
$ cd /home/<project folder>/
$ source devel/setup.basha
$ cd roslaunch ball_chaser ball_chaser.launch
```

#### Optionally view the RGB camera feed, in a new terminal enter:
```sh
$ cd /home/<project folder>/
$ source devel/setup.basha
$ rosrun rqt_image_view rqt_image_view  
```

