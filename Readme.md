## What is this package for?
This package is for utilizing ros2 px4 gps drone control to go to location near red pool from home pos., detect its (red pool) center position, fly down, fly up, and after that return to home pos.

## What you need to install this package:
- ROS2 Humble  
- ubuntu 22.04 jellyfish   
- px4-autopilot (with either gz_x500 or gazebo-classic [depends on your system])  
- micro-ros-agent or Micro-XRCE-DDS-Agent  
- ros-humble-ros-gz-bridge (for subscribing camera topics)  
- rqt_image_view  
- opencv v4.10.0  
- gazebo garden (NOT HARMONIC!)  
- ros_gz_bridge (this one after installing gz garden)  

## most important things (setting things up) AUTOMATIC INSTR.
0) Go to launch/hydrolab_launch and find a line:  
    #after what time after starting 
    time_start = 20.0  
You need to then propably (if you have pc better than 3060 and ryzen 7) lower this time or leave it be or if you have worse specs you need to make this time bigger. Choose it by trial and error method. Its basically the time after which the nodes will start bo they must start after each essential subprocess start thats why this time!  
After setting time you need to copy world betterpool from worlds folder and paste it into:  
    ~/PX4-Autopilot/Tools/simulation/gz/worlds/  
and also copy and paste basement tank into:  
    ~/PX4-Autopilot/Tools/simulation/gz/models/  
However you can choose your own world then you would need to change world name in topics in all of the code to your world name and also change some settings in launch for your specific world. After that you just need to add lidar config to monocamdown one. You can do it by replacing 
```
<include merge='true'>
      <uri>x500</uri>
</include>
```
in PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam_down/model.sdf by:
```
<include merge='true'>
      <uri>x500_lidar_down</uri>
</include>
```

1) basically after doing all steps above you just need to build it, source it and launch it, so:  
`colcon build --packages-select hydrolab`  
`source install/setup.bash`  
`ros2 launch hydrolab hydrolab_launch.py`
> **Note**  
> *IT'S VERY IMPORTANT YOU GIVE THE NAME HYDROLAB TO PACKAGE CAUSE I CONFIGURED ALL THE SETUPS / LAUNCHS FOR THIS PACKAGE NAME!!!*

  
## most important things (setting things up) MANUAL INSTR. (if automatic startup doesn't work for ya)
1) for the microrosagent to work gazebo need to be running already (in px4-autopilot folder (after copying from github: ```https://github.com/PX4/PX4-Autopilot.git```) type:  
```PX4_GZ_WORLD=betterpool PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4``` to start x500 model with downward facing monocam & lidar and world with one aruco. *NOTE YOU NEED TO BE IN PX4 DIRECTORY!* 
You can always also start new world by typing  
```make px4_sitl gazebo``` or ```make px4_sitl gz_x500```    
after that you should see gazebo is being opened. Note gz_x500 is what i tested this package on, gazebo-classic is older and usage of this package may differ a little if you try it there. I added automatic takeoff so:    
`comander takeoff`  
is no longer needed to start moving the drone (typed in gazebo console).  
> **Note**  
> takeoff sequence is unfortunately not very logical I basically copied some code from doc examples and changed it to fit program goals. Curr start sequence is: offboard, trajectory, cmd set mode, arm, offboatrd, trajectory  
2) after that open microrosagent by typing:   
```micro-ros-agent udp4 --port 8888```   
or while being in Micro-XRCE-DDS-Agent folder: (we used MicroXrce btw look under)  
```Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888```  
After that you should see some topics appear (```ros2 topic list```) on which you may publish some info to simulation i gazebo.  
3) (currently) you also need to type:  
```
ros2 run ros_gz_bridge parameter_bridge /world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image
```  
to get the image and  
```
ros2 run ros_gz_bridge parameter_bridge /world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```  
to get the camera info.  
```
ros2 run ros_gz_bridge parameter_bridge /world/betterpool/model/x500_mono_cam_down_0/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```  
To add ros-gazebo bridge for being able to subscribe from cam module (ultimately being able to detect aruco)  
After starting the bridge you can check whether you can see sth on /camera, just type:  
```ros2 run rqt_image_view rqt_image_view```  
and check topic: `/world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image`  
> **NOTE!**  
> **/world/betterpool/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image topic name may differ if you dont have gazebo garden! For best comaptibility I recommend uninstalling your current gazebo and ros_gz_bridge and installing gazebo garden and after that installing new ros_gz_bridge. This may also solve the problem of not seeing any data on these topics after subscribing to them! after that you should type gz topic -l this should show what topics you curr have in your gazebo (this you can also check in gazebo)**  
4) having gazebo and microros opened you should build and run node downlaod from this prj. (I mean hydrolab). So you should type (in your ros2 ws NOT IN YOUR SRC!):  
```colcon build --packages-select hydrolab```  
```source install/setup.bash``` (or .zs, or any other depends on what console type you have)  
```ros2 run hydrolab drone_controller```  
5) drone should fly from A to B.  

## Sth important bout the code (drone controller (main program))
1) Currently the callback list for the program is like this (only intial callbacks are listed cause in the loop):  
position_callback (setting position states for SM) ->   
-> aruco_callback (calculatin average position of aruco) ->   
-> timer_callback (arming the drone (at the beginnning), setting correct   offboatrd mode (publish_offboard_control_mode (for state 3 just setting   velocity control, and for others position mode)) for certain states, publishing setpoints (publish_setpoint (by position publishing setpoints although not in state 3))).  

## Sth important bout the launch file (hydrolab_launch)
1) Currently it's this queue of subprocesess callbacks:  
px4 gazebo -> micro xrce agent -> camera_image_bridge, camera_info_bridge (custom bridges agent) -> lidar_bridge -> drone_controller, aruco_tracker  

## Currently I dont give a damn bout license for this if you find me you can buy me a coffe :)