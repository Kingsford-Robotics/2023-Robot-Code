# 2023-Robot-Code

## Subsystems:
<ul>
<li>Swerve Drivetrain</li>
<li>Turntable</li>
<li>Intake</li>
<li>Arm</li>
<li>Elevator</li>
<li>Vision</li>
</ul>

## Deployment:
All code in this repository is meant to be directly deployed to the roboRIO. All required dependencies should be included if cloned. 
<br> The code utilizes data from a Jetson Xaviervcoprocessor equipped with a ZED Mini stereocamera and USB camera. 
<br> Data from coprocessor is sent over network tables. A Limelight 2.0 is used in the same way.
<br> The vision code for the Jetson Xavier can be found in this repository: https://github.com/Kingsford-Robotics/2023-Jetson-Code
