
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

Now then, running on Ubuntu 22.04 LTS using ROS-Humble we went through a series of processes in order to organizer this into a simple manner. Although, we will note that they're some poor practices as a result of time constraints and it being our first time working with the ROS framework. These will be noted and clarified furthermore in this document. Clifford is an entirely 3D printed project that uses inverse-kinematics in order simulate a dog walking.

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

#### For Launching & Booting Clifford

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

#### Calculating Battery Life 
Using the 5V pin provided to power the ADS1150, we needed to step down the balance charger voltage outputted from the LiPo. We created a small PCB board which included a voltage divider circuit such that the highest voltage read would be ~5V. Now with voltage being able to be read, we then determined the highest voltage ie @ 100% and at 0%. We then used the current voltage in order to calculate the current battery percentage.

#### Other Useful Components  
After finalizing the project here our somethings that help improve production. 
  - Buy a dedicated LiPo Battery Balance Charger, these will guarantee the quality of battery being used and can charge relatively quick.
  - Using a DC Power Supply, would prevent the need of constantly recharging the battery after testing.
  - If possible maximize the hardware you use. We settled on the Raspberry Pi 4B, it was okay. I figured I would do most of my programming on the Pi anyway but there were times it was just so slow.
  - Get a dedicated cooler for your controller, especially for Raspberry Pi. These things will heat up quickly.






## Software

## Kinematics

## Future Additions

## Conclusion

## Bill of Materials

## References
