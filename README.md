# UAV-uGV-system
![coverimage](https://github.com/scifiswapnil/UAV-uGV-system/blob/main/images/first-prototype.jpeg)

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


## Bill of materials

| **Sr No** 	| **Part Name** 	| **Link** 	| **Spec** 	| **Cost** 	| **Quantity** 	| **Total**<br>(in INR) 	|
|:---:	|:---:	|:---:	|:---:	|:---:	|:---:	|:---:	|
| **1** 	| Microgear Motor 	| [source](https://www.mgsuperlabs.co.in/estore/210-1-Micro-Metal-Gearmotor-LP-6V-with-Extended-Motor-Shaft?tag=LP) 	| 210:1 gear ratio<br>60 RPM<br>1.9 kg-cm<br>Extended shaft 	| 1679 	| 5 	| 8395 	|
| **2** 	| Microgear Motor Encoder 	| [source](https://www.mgsuperlabs.co.in/estore/Sensors/Encoders/Magnetic-Encoder-Pair-Kit-for-Micro-Metal-Gearmotors-12-CPR-2-7-18V) 	| 12 CPR<br>LP & HP compatible 	| 999 	| 5 	| 4995 	|
| **3** 	| Microgear Wheels 	| [source](https://www.mgsuperlabs.co.in/estore/Robotics/Wheels/Pololu-Wheel-42x19mm-Pair) 	| 42mm x 19mm Wheel Rugged<br>encoder Built in 	| 700 	| 5 	| 3500 	|
| **4** 	| Rpi waveshare mini cam 	| [source](https://www.mgsuperlabs.co.in/estore/Raspberry-Zero-V1.3-mini-Camera) 	| Rpi zero 5 MP mini camera 	| 1979 	| 1 	| 1979 	|
| **5** 	| 9DOF IMU sensor 	| [source](https://www.mgsuperlabs.co.in/estore/Sensors/Motion-Position-Sensors/Grove-IMU-9-DOF?sort=p.price&order=DESC) 	| I2C interface <br>400kHz Fast Mode I2C 	| 1479 	| 1 	| 1479 	|
| **6** 	| Motor Driver 	| [source](https://www.mgsuperlabs.co.in/estore/Robotics/Motor-Drivers/DRV8833-Dual-Motor-Driver-Carrier) 	| Dual HBridge driver<br>1.2A per channel 	| 489 	| 2 	| 978 	|
| **7** 	| Li-ion BMS 	| [source](https://robu.in/product/3s-12v-18650-lithium-battery-protection-board-12v-10a-overcharge-overcurrent-protection/?gclid=Cj0KCQiAsdHhBRCwARIsAAhRhsm5FwV4sGKYPKlrcEu-bcQJZBrBzpFeIFE3jpBhRJywKtu3QIwOPJsaAoluEALw_wcB) 	| 3S Li-Ion charge/Discharge <br>11.1 - 12.6 CutOff 	| 200 	| 2 	| 400 	|
| **8** 	| Li-ion battery 	| [source](https://www.amazon.in/UltraFire-Battery-Li-ion-Rechargeable-Charger/dp/B07BK1KRC1) 	| 2 pcs 18650<br>charger included 	| 3529 	| 2 	| 7058 	|
| **9** 	| ESP 32 + camera 	| [source](https://www.banggood.in/TTGO-T-Journal-ESP32-Camera-Development-Board-OV2640-SMA-WiFi-3dbi-Antenna-0_91-OLED-Camera-Board-p-1379925.html?cur_warehouse=CN) 	| ESP32<br>0.9 MP Camera  	| 1060 	| 1 	| 1060 	|
| **10** 	| Rpi zero camera kit 	| [source](https://www.mgsuperlabs.co.in/estore/Raspberry-Pi-Zero-v1.3-Camera-Pack-Includes-Pi-Zero) 	| RPi zero<br>case<br>convertors 	| 2490 	| 1 	| 2490 	|
|  	|  	|  	|  	|  	| **Total** 	| 32334 	|


## Existing uGV platform comparison

| **Sr. No.** 	|  	| **1** 	| **2** 	| **3** 	| **4** 	|
|:---:	|:---:	|:---:	|:---:	|:---:	|:---:	|
| **Robot name** 	|  	| Pololu Zumo  	| Nvidia Jetbot Ai 	| Turtlebot Burger 	| Robomax(Ours) 	|
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


PCB prototype             |  UGV CAD Model | First prototype
:-------------------------:|:-------------------------:|:-------------------------:
![](https://github.com/scifiswapnil/UAV-uGV-system/blob/main/images/PCB.jpeg)  |  ![](https://github.com/scifiswapnil/UAV-uGV-system/blob/main/images/agv.png) |  ![](https://github.com/scifiswapnil/UAV-uGV-system/blob/main/images/miniUGV.jpg)
