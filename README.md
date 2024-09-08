
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
    <td> Arducam 16MP Wide Angle USB Camera </td>
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
    <td>Sunfounder LCD1602 16x2 </td>
</tr>
 
</table>

#### Communication Protocol 
As we were using multiple third-party modules such as servo driver we utilized I2C bus provided on the Raspberry Pi 4B. This worked perfectly but didn't allow us to have multiple devices using I2C, as we had a few more. Creating a simple PCB using header pins we created an I2C bus expander that allowed all devices to meet at a single point in order to communicate with the Pi.

#### FPV Camera 
Using motion libraries we were able to set up a first-person live-streaming feed on a local network. By dedicating a specified port to live-streaming (8081) simply use the IPV4 address followed by the port like so xxx.xxx.xxx.xxx:8081 to access from any device on the same network. 
To set the config file for the motion service to work navigate to:
```
sudo nano /etc/default/motion
```
Make the following changes to the config file:
```
daemon on
stream_localhost off
webcontrol_localhost off
framerate 1500
stream_quality 100
quality 100
width 640
height 480
post_capture = 5
```
After that navigate to the default file:
```
sudo nano /etc/default/motion
```
Make sure the file contains the following if you want live streaming to start when turning on the Pi.
```
start_motion_daemon=yes
```
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

Using the left joystick, you can determine the direction of movement (forward or backward). This is handled through flags that check the joy topic axes sign (-/+). Using the Joy topic callback we are able to constantly update Clifford's relative position. Clifford's walking consists of 4-point leg motion. However, the legs move in pairs, meaning the front left and back right move together, and the front right and back left move together. The overall structure of the walking gait was as follows, a single set will take 'charge' meaning they're the legs moving forward while the others are dragging backwards creating the ability to go forward. The speed at which the legs drag backward is based on the input of the user, and timing so that a pair of legs complete a single point (the ones dragging backwards) while the other completes three 3 (the other set). This allows us to constantly have two points of contact with the ground at all times.



The leg motion follows a square pattern, making the programming straightforward. At each position, the program checks whether the leg is moving forward or backward and its current relative position (x, y, z). Only one coordinate (either x or y) changes at a time. Multiple conditions verify if the leg has passed or reached its target coordinate, then update the target and current indices. While most of the programming remains consistent, the conditions for each leg differ slightly.

#### Sample Code
```
     if forward and self.front_right_current[2] >= self.front_right_target[self.set1_target_index][2] or \
                        (not forward and self.front_right_current[2] <= self.front_right_target[3][2]):
                            self.update_servos()
                    else:
                        if forward:
                            self.front_right_current[2] = self.front_right_target[self.set1_target_index][2]
                            self.back_left_current[2] = self.back_left_target[self.set1_target_index][2]

                            self.set1_target_index = 2 
                            self.set1_walk_index = 1
                        else:
                            self.front_right_current[2] = self.front_right_target[self.set1_walk_index][2]
                            self.back_left_current[2] = self.back_left_target[self.set1_walk_index][2]
                            self.set1_walk_index = 3
                            self.set1_target_index = 0
                       
                        if not forward:
                            self.front_left_current[0] = self.front_left_target[self.set2_walk_index][0]
                            self.back_right_current[0] = self.back_right_target[self.set2_walk_index][0]
                            self.set2_walk_index = 3
                            self.set2_target_index = 0
```

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

<table>
  <tr>
    <th>Component</th>
    <th>Quantity</th>
    <th>Link</th>
    <th>Total Cost</th>
  </tr>
  <tr>
    <td>DS3225MG 25 Kg Servos x2</td>
    <td>6</td>
    <td><a href="https://amzn.to/4ej0M4R">Link</a></td>
    <td>$264</td>
  </tr>
  <tr>
    <td>LiPo Battery Charger</td>
    <td>1</td>
    <td><a href="https://amzn.to/3TkDkMs">Link</a></td>
    <td>$51</td>
  </tr>
  <tr>
    <td>T deans T branch (Parallel) x2</td>
    <td>1</td>
    <td><a href="https://amzn.to/3XDdNQ7">Link</a></td>
    <td>$13</td>
  </tr>
  <tr>
    <td>Wire Strippers</td>
    <td>1</td>
    <td><a href="https://amzn.to/3XigNB1">Link</a></td>
    <td>$14</td>
  </tr>
  <tr>
    <td>18 Gauge Wire 70 ft</td>
    <td>1</td>
    <td><a href="https://amzn.to/3zb0m1m">Link</a></td>
    <td>$20</td>
  </tr>
  <tr>
    <td>USB-C Charging cable x2</td>
    <td>1</td>
    <td><a href="https://amzn.to/3TlbYGa">Link</a></td>
    <td>$14</td>
  </tr>
  <tr>
    <td>7.4V 6200mAh Battery x2</td>
    <td>1</td>
    <td><a href="https://www.amazon.ca/Zeee-Battery-6200mAh-Connector-Vehicles/dp/B0868FM5S4/ref=sxin_15_pa_sp_search_thematic_sspa?content-id=amzn1.sym.ea4b7f00-c440-4d65-bd3f-caa76cb13654%3Aamzn1.sym.ea4b7f00-c440-4d65-bd3f-caa76cb13654&crid=33LM2212JB495&cv_ct_cx=7.4+v+battery&dib=eyJ2IjoiMSJ9.u1_mUXoAXQUkLqwlo6JmZ_JasDNoLpM2zjLmHqhT1SXFAS3gFbbyExDEEVSfDnU-VrqQrmxrPprw-hwRv47i2g.X91cB13EpN38YKRBIQq6b7JR4TpyyQxxQAy_Xtx-7m4&dib_tag=se&keywords=7.4+v+battery&pd_rd_i=B0868FM5S4&pd_rd_r=e0d6c371-ae9a-44c2-9644-28ec4dc6a87b&pd_rd_w=YCerP&pd_rd_wg=LZZ1P&pf_rd_p=ea4b7f00-c440-4d65-bd3f-caa76cb13654&pf_rd_r=CW9N2R8GTMZPJEF1FTYW&qid=1719076154&sbo=RZvfv%2F%2FHxDF%2BO5021pAnSA%3D%3D&sprefix=7+4v+battery%2Caps%2C114&sr=1-4-acb80629-ce74-4cc5-9423-11e8801573fb-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9zZWFyY2hfdGhlbWF0aWM&psc=1">Link</a></td>
    <td>$60</td>
  </tr>
  <tr>
    <td>RPI Heatsink</td>
    <td>GeeekPi Aluminum Heatsink</td>
    <td><a href="https://www.amazon.ca/GeeekPi-Raspberry-Aluminum-Heatsink-Controllable/dp/B091L1XKL6/ref=sr_1_3?crid=2AU85VCUAN29T&dib=eyJ2IjoiMSJ9.GSSZFPpAj7yoqb1p7UurtBXisj_cTUKNFNwfdtqQhAs7eLEE4wIwg9kaVFt_mQtI88CDidB4luhWGcJYOWUP3vQkWmHwf6Lm3UQyMUJ2S4Raf2kk1-71W9Par1XX9xk7eskqMudFeE5TvZPQjz-4Hh8c3_L62yVVhQhdh1XC5J7oQiWjDbT-WMkVdq_k4nep6Lelzx84Aveqi0-g-h-qHyqxcRksAwqkT3E1nMwnhU8bKAZKNiGL6u8Ec3CQXdxwMO7dPURM9rW1WKRBBWZwdwpzPx_7MoLBuVO4Q1OV98E.srppl_jugzK74knNIOraYc91CJ87GKbXOUnzMmdSt1k&dib_tag=se&keywords=geeekpi+aluminum+heatsink&qid=1719076248&s=electronics&sprefix=geekpi+alunimum+heatskink%2Celectronics%2C94&sr=1-3">Link</a></td>
    <td>$16</td>
  </tr>
  <tr>
    <td>HDMI to Micro HDMI</td>
    <td>UGREEN HDMI to Micro HDMI</td>
    <td><a href="https://www.amazon.ca/UGREEN-Ethernet-Support-Resolution-Tablets/dp/B015GR44CG/ref=sr_1_8?crid=2US4JRVXB2XJL&dib=eyJ2IjoiMSJ9.Ig1BvlBz5seDeJ9TbfZz76HgAzLr2H-icf_hnmTC53lbnN0U-Wv4zdTopehEV6tVPIpiTY2NZown5N_c_UYPGIvgsPSvyl6K7SDVnxQnUdIsWPTOg27hYpUNsEdoyuncVyDdxWUhu_HDA-9OZRqeI9k851N65tNgyFzW7y9iT4uMTnWFqmNsyhTaGMEfPD0zSTaH8V3dfTYO05_7G7pwMA7zRSgODW55_cqLs2XdAcZ7EzwwIwT2LvqgkCWsYQl55p_LZAkOn8b5IJJocVXCpME2otCzZOJGXFDeY_bkmWU.rX9-deHLJNbmFjh6hfL7E101zCSXccWak7PFnlbqdhI&dib_tag=se&keywords=hdmi%2Bto%2Bmicro%2Bhdmi&qid=1719076299&s=electronics&sprefix=hdmi%2Bto%2Bmicro%2B%2Celectronics%2C102&sr=1-8&th=1">Link</a></td>
    <td>$14</td>
  </tr>
  <tr>
    <td>DROK RPI Buck Converter (used as spare)</td>
    <td>DROK DC-DC Buck Converter</td>
    <td><a href="https://www.amazon.ca/Adjustable-Converter-Stabilizer-Regulator-Protective/dp/B01FQH4M82/ref=sr_1_11?crid=193IE8HBH2QIO&dib=eyJ2IjoiMSJ9.OuMyHn-AXkWX_SJ8wNQ3KsNMUkccBZv-3vmMqrYAxBZkc2aemW4gGhTsm8FsMP3MYWimZZ8_PZLsgNsrE68hsA08zK6xp_7XVNRDFajHkE67wsNJcUAORz0ArAgjDYNNiNi5VH5MzsYs_-RXWghFsm7eK_NqHsz6g0mBtgReWpRby6fVx1T7TIOYOjm1Je-KiO-aP1LMmRcPDec4AXYiEYJwF_lmAaC2vzZB7_f8Qt_hFaUpt_m-xSFTiC2kOuOurgJgy4cf84xQaMFdCIK4zmrz8brqNi0iek-huNiTVoQ.uX2nvFimUALNmQGE042Ku_06VOQqFYEWmWTGQFyX_9U&dib_tag=se&keywords=drok+dc+to+dc+buck+converter&qid=1719076360&s=electronics&sprefix=drok+dc+to+dc+buck+converter+%2Celectronics%2C88&sr=1-11">Link</a></td>
    <td>$26</td>
  </tr>
</table>

## References
