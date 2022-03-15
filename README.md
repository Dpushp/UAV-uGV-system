# UAV-uGV-system
![coverimage](https://github.com/scifiswapnil/UAV-uGV-system/blob/main/agv.png)

**Aim**: To design and evalute a cooperative planning strategy for payload transportation using a heterogeneous swarm of robots.

**Challenges:**
- Design a UAV-UGV system physically coupled using a passive tether; % to explore unknown regions hidden from the field of view of the UAV like cavities and regions where UGV alone cannot reach;
- Design a miniature robot with autonomous navigation capabilities;
- Design and implement exploration and manipulation techniques for the miniature robot, as it might not be able to use existing algorithms because of the lack of (high-end) sensors and computations required;
- Design a miniature robot with autonomous navigation, exploration and manipulation capabilities, even though the miniature robot is equipped only with limited low-end sensors and computation board;
- Design a gripper that can be installed on a mini robot to pick and place objects in a hidden space;
- Design a robust software-system architecture to support repetitive operations.

## System Dependencies
- Ubuntu : 16.04+
- ROS : Kinetic+
- CPU : ARM7+ or Intel i3+ or AMD R4+
- RAM : 4GB+
- Memory : 10GB+

## Setup and installation
```bash
cd ~/
git clone https://github.com/scifiswapnil/UAV-uGV-system
cd UAV-uGV-system/ROS
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
roslaunch robomax_description robomax_world.launch 
```

## Todo
- [x] Tether control mechanism 
- [x] uGV exploration strategy 
- [x] Rope simulation 
- [x] global planning strategies 
- [x] Path planning tests
- [x] uGV prototyping
- [x] uGV PCB design and prototyping 
- [ ] Benchmarking **to be disscussed**

## Existing uGV platform comparison

| **Sr. No.** 	|  	| **1** 	| **2** 	| **3** 	| **4** 	|
|:---:	|:---:	|:---:	|:---:	|:---:	|:---:	|
| **Robot name** 	|  	| Pololu Zumo  	| Nvidia Jetbot Ai 	| Turtlebot Burger 	| Robomax 	|
| **Structural** 	| Weight 	| 160 grams 	|  	| 900 grams 	| 350 grams 	|
|  	| Size 	| 100x100x45 mm 	|  	| 138 x 178 x 192 mm 	| 130 x 120 x 55 mm 	|
|  	| Drive type 	| Differential with <br>track drive 	|  	| 2 Differential drive<br> + 2 castors 	| 4 wheel drive 	|
| **Sensors** 	| Monocular camera 	| No 	| 8 MP ; 30/60 FPS ; <br>Monocular camera 	| No (can be added) 	| 5 MP ; 30/60FPS ; <br>Monocular camera  	|
|  	| IR 	| 2 front ; 1 left ; <br>1 right 	| No (can be added) 	| No (can be added) 	| 2 front 	|
|  	| LIDAR 	| No  	| No  	| Yes 	| No  	|
|  	| IMU 	| 6 DoF IMU 	| No (can be added) 	| 9DoF IMU 	| 6 DoF IMU 	|
| **Communication** 	|  	| No  	| Wifi/Bluetooth 	| Wifi/Bluetooth 	| Wifi/Bluetooth 	|
| **Actuator / Display** 	|  	| Buzzer / OLED<br> display unit 	| OLED Display  	| No 	| EPM - electro-<br>permanent magnet 	|
| **Computation power** 	|  	| Atmel <br>microcontroller 	| Cortex-A57 + <br>128 Core GPU 	| Rpi 3 b + OpenCR  	| Rpi zero W + ESP 32 	|
| **Battery type** 	|  	| 4x AA Alkaline<br> battery  	| Li-ion/Li-Po/Portable <br>power Bank 	| Lipo Battery 	| 2x 18650 Li-ion <br>battery 	|
| **Operation time** 	|  	| 45 mins  	| 65-90 mins  	| 85-110 mins 	| 30 mins 	|
| **Programmable** 	|  	| Arduino platform 	| Debian platfrom / ROS 	| Debian platform / <br>Arduino / ROS 	| Debian platform / <br>ESP-IDF / ROS 	|
