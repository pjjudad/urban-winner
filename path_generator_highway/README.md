# Path Generator

This ROS package is an open source version of planning package from BITFSD, which is built for generating the reference path.  

There are three methods in the path generator, each matched with a specific competition mission. 

## Important Dependencies

* catkin_tools
* ROS

## ROS topic

### Acceleration Mission

* **Subscriber**
  * /estimation/slam/state  //这个都有订阅
  * /planning/end_point  //给个终点即可
* **Publisher**
  * /control/pure_pursuit/control_command   //如果要用MPC控制，这个是否需要将发布pure_pursuit命令改为发布MPC命令?

### Skidpad Mission

* **Subscriber**  
  * /estimation/slam/state   //这个都有订阅
  * /transform_matrix    //矩阵的方式
* **Publisher**
  * /control/pure_pursuit/control_command

### **Trackdrive Mission**

 Acceletation和Skidpad应该也用了 /map吧，rqt_graph里面都有显示，而且在实际运行的时候在车辆正常动之前都有提示 Set Local Map OK    
* **Subscriber**
  * /map                     //里面是锥桶在局部地图的位置和颜色  其他两个用到 、map了没？
  * /estimation/slam/state    //这个都有订阅
* **Publisher**
  * /control/pure_pursuit/control_command

## Step

### Prepare

```bash
# Build your workspace
catkin build

# Source environment
source devel/setup.bash
```

### Run simulation

Please refer to this [Repository](https://github.com/bitfsd/fssim) .

### Launch

```bash
# For Trackdrive Mission
roslaunch path_generator trackdrive.launch

# For Acceleration Mission
roslaunch path_generator acceleration.launch 	

# For Skidpad Mission
roslaunch path_generator skidpad.launch						
```

## Parameters

In the `./config` folder, we provide three `.yaml` files. Each file contains different parameters for the three FSAC competitions respectively.

* **Subscriber:** List the ros topics which the code will subscribe.
* **Publisher:** List the ros topics which the code wil publish.
* **Trajectory params:** Describe the parameters related to the generation of trajectory.
* **mission:** You can choose the mission here.
* **simulation:** If you are running in [simulator](https://github.com/bitfsd/fssim), make sure this parameter is true, or it will be a car crash in skidpad match.

## Note:

* Topic `/transform_matrix` comes from [skidpad detector](https://github.com/bitfsd/fsd_algorithm/tree/master/ros/planning/skidpad_detector).
* Topic `/planning/end_point` comes form [line detector](https://github.com/bitfsd/fsd_algorithm/tree/master/ros/planning/line_detector).