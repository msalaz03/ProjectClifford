
# Project Clifford


* [Overview](#overview)
* [Setup](#setup)
* [Hardware](#hardware)
* [Software](#software)
* [Kinematics](#kinematics)
* [Future Additions](#future-additions)
* [Conclusion](#conclusion)
* [Bill of Materials](#bill-of-materials)
* [References](#references)

## Overview (Please Read)
Over the past four months Cameron, Chase and I tried tackling our own version of Boston Dynamic's Spot. This repository contains the source code that interfaces all hardware interactions between our small red friend. The software includes prebuilt libraries designed for the Raspberry Pi and also references other projects similar to ours. 

Now then, running on Ubuntu 22.04 LTS using ROS-Humble we went through a series of processes in order to organizer this into a simple manner. Although, we will note that they're some poor practices as a result of time constraints and it being our first time working with the ROS framework. These will be noted and clarified furthermore in this document. Clifford is an entirely 3D printed project that uses inverse-kinematics in order simulate a dog walking. Clifford also has FPV camera mounted which can be accessed through any web browser provided you're on the same network as him.

Please check the references, this would have not been possible without open-source projects. 

## Setup
The organization of nodes is not ideal. However, with time it can be reorganized neatly. Currently we have mostly everything inside a single node. For anyone attempting to recreate anything similar please take the time to do so prior to programming the entire project.

```
Three most important folders for this project.

projectclifford_ws/
│
├── src/
│   ├── clifford_sim_1
│   │   └── ...
│   ├── servo_driver
│   │   └── ...  
│   └── ...
│   ├── teleop_controller
│   │   └── ...  
│   └── ...
```

#### For Using RVIZ With Clifford
In order to test our kinematic equations more efficiently we used RVIZ to visualize what is happening when there is a controller input. 'clifford_sim_1' contains a URDF file for the final version of Clifford, exported through SolidWorks URDF exporter. The launch file within this directory only launches the URDF file into RVIZ and does not contain RobotStatePublisher by default. You can enable joint-state-publisher-gui by uncommenting the related lines within the launch file or can use additional launch file that waits for joy (controller input) and joint-state-publisher topic. The second directory relative to running RVIZ is 'teleop_controller' which creates topic for joint-state-publisher.

**NOTE:** This totally could have been organized into a single launch file but wasn't due to circumstances at the time.

In order to launch please do the following.

```
ros2 launch clifford_sim_1 clifford_sim_1.launch.py
ros2 launch teleop_controller Clifford_full_model.launch.py
```
#### For Launching & Booting Clifford

To physically launch clifford is pretty simple (as long as you have all required dependecies), simply run.

**Note:** At this point Clifford cannot physically turn as the turning motion was not programmed, however the kinematic equations needed to do are solved and within the software.
```
ros2 launch servo_driver joy_servos.launch.py
```

If you want don't want to follow Clifford, you can see where he is through livestreamed footage. First identify the IPV4 address of clifford and type the following.
```
xxx.xxx.xxx.xxx:8081
```

## Hardware
From researching similar projects to Clifford and doing our research we sourced components that we thought were optimal for this project. Consisting of two voltage regulators to isolate the power going into the servo driver and Raspberry Pi. ADC boards in order to measure different voltages levels into a relative percentage for battery life. A high-capacity battery in order to prolong battery life. LCD screen in order to demonstrate battery life real-time. 25 KG servos, a mixture between torque and speed in order to ensure smooth movements. 

<br>
  
<table>
  <tr>
    <td> <b>Electrical Component </b></td>
    <td>  <b>Model Name </b>  </td>
  </tr>

  <tr>
    <td>Controller</td>
    <td>Raspberry Pi 4B</td>
  </tr>

  <tr>
    <td> Camera </td>
    <td>  no information rn </td>
  </tr>
  
   <tr>
    <td>Voltage Regulator for Pi</td>
    <td>LM2596</td>
  </tr>

 <tr>
    <td>Voltage Regulator for Servos</td>
    <td>DROK DC-DC High Power </td>
  </tr>

 <tr>
    <td>LiPo Battery</td>
    <td>Zeee 2S LiPo 6200mAh @ 7.4V</td>
</tr>

 <tr>
    <td>Servos</td>
    <td>DS3225MG</td>
</tr>

 <tr>
    <td>Servo Driver</td>
    <td>PCA9685</td>
</tr>

 <tr>
    <td>ADC Board</td>
    <td>ADS1115</td>
</tr>

 <tr>
    <td>Gyroscope</td>
    <td>MPU-6050</td>
</tr>

 <tr>
    <td>LCD 16x2</td>
    <td>No information rn</td>
</tr>
 
</table>

#### Communication Protocol 
As we were using multiple third-party modules such as servo driver we utilized I2C bus provided on the Raspberry Pi 4B. This worked perfectly but didn't allow us to have multiple devices using I2C, as we had a few more. Creating a simple PCB using header pins we created an I2C bus expander that allowed all devices to meet at a single point in order to communicate with the Pi.

#### FPV Camera 
Using motion libraries we were able to set up a first-person live-streaming feed on a local network. By dedicating a specified port to live-streaming (8081) simply use the IPV4 address followed by the port like so ***.***.***.***:8081 to access from any device on the same network. 

#### Calculating Battery Life 
Using the 5V pin provided to power the ADS1150, we needed to step down the balance charger voltage outputted from the LiPo. We created a small PCB board which included a voltage divider circuit such that the highest voltage read would be ~5V. Now with voltage being able to be read, we then determined the highest voltage ie @ 100% and at 0%. We then used the current voltage in order to calculate the current battery percentage.

#### Other Useful Components  
After finalizing the project here our somethings that help improve production. 
  - Buy a dedicated LiPo Battery Balance Charger, these will guarantee the quality of battery being used and can charge relatively quick.
  - Using a DC Power Supply, would prevent the need of constantly recharging the battery after testing.
  - If possible maximize the hardware you use. We settled on the Raspberry Pi 4B, it was okay. I figured I would do most of my programming on the Pi anyway but there were times it was just so slow.
  - Get a dedicated cooler for your controller, especially for Raspberry Pi. These things will heat up quickly.


## Software
The software went through many revisions, currently all software is under a single node, 'servo2_pca9685' within the directory 'servo_driver'. This includes the various kinematic equations needed in order to move each leg. The ultrasonic sensor can also be found within this file to override a controller input if Clifford detects he is too close to an object.

#### PS4 Joystick
Using the left joystick, you can determine direction of movement (forward or backward). This is handled through flags that check the joy topic axes sign (-/+). Using the Joy topic callback we are able constantly update Clifford his relative position. Cliffords walking consist of 4-point leg motion. However the legs move in 2's, meaning front left and back right move together and then front right and back left move together. 


#### PS4 Buttons

Various buttons are configured for the following. During this time it changes a flag so that clifford knows that he is no longer waiting for 'walk' commands. Additionally, for any third party expanders 
  - Lay down or stand tall.
  - Lean side to side.
  - Lean forward and backward.
  - Reset to default stance.
  - Shutdown Clifford.


## Kinematics
- Pictures would be nice
- Include design of gait.
- include references here and also in references.

## Future Additions

## Bill of Materials

## References
