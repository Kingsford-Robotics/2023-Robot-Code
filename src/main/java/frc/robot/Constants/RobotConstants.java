// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class RobotConstants {
    /*Sensors*/
    public static final int pigeonID = 1;

    public static final class ArmConstants {
        public static final int armMotorID = 12;
        
        /*Arm Pneumatics*/
        public static final int armExtensionSolenoidFWD = 1;
        public static final int armExtensionSolenoidREV = 0;

        public static final int armGrabSolenoidFWD = 2;
        public static final int armGrabSolenoidREV = 3;

        /*Arm Encoder Values*/
        public static final int armEncoderID = 30;
        public static final double armEncoderOffset = 350.0;

        /*Arm Motion Magic Constants*/
        private static final double armMaxSpeed = 6380.0 * (2480.0 / 600.0); //Max speed in sensor units per 100ms
        private static final double armPercentSpeed = 0.5; //Percent of max speed to cruise at.

        public static final double armCruiseVelocity = armMaxSpeed * armPercentSpeed; //Cruise velocity in sensor units per 100ms 
    
        private static final double armTimeToMaxSpeed = 0.5; //Time to reach cruise velocity in seconds
        public static final double armMaxAcceleration = armCruiseVelocity * 1.0 / armTimeToMaxSpeed;   //Max acceleration in sensor units/100ms per second

        public static final int armSCurveStrength = 3;

        //TODO: Set to minimum value that allows arm to move at horizontal.
        /*Arm Gravity Compensation*/
        public static final double armMaxGravityComp = 0.05;    //Percent output

        /*Arm PID Constants*/
        //TODO: Tune PID values using Phoenix tuner.
        public static final double armKp = 0.0;
        public static final double armKi = 0.0;
        public static final double armKd = 0.0;

        //1023 is the max output of the motor controller.
        //kF = (PercentOutput X 1023) / Speed(Encoder Units/100ms)
        public static final double armKF = 1.0 * 1023.0 / armMaxSpeed;    //TODO: Find actual speed at lower output percent for free motor.

        /*Arm Gear Ratio*/
        public static final double armGearRatio = 90.0 * (72 / 22.0); //TODO: Check angle to make sure 90 degrees is correct.

        /*Arm Limits*/
        //Horizontal is 0, up is negative.
        public static final double armMaxAngle = 180;
        public static final double armMinAngle = -10;
    }
  
    public static final class ElevatorConstants{
        public static final int elevatorMotorID = 11;

        /*Limit Switches*/
        public static final int elevatorTopLimitSwitchID = 0;
        public static final int elevatorBottomLimitSwitchID = 1;

        //Elevator Motion Magic Constants
        private static final double elevatorMaxSpeed = 6380.0 * (2480.0 / 600.0); //Max speed in sensor units per 100ms
        private static final double elevatorPercentSpeed = 0.5; //Percent of max speed to cruise at.

        public static final double elevatorCruiseVelocity = elevatorMaxSpeed * elevatorPercentSpeed; //Cruise velocity in sensor units per 100ms 
    
        private static final double elevatorTimeToMaxSpeed = 0.5; //Time to reach cruise velocity in seconds
        public static final double elevatorMaxAcceleration = elevatorCruiseVelocity * 1.0 / elevatorTimeToMaxSpeed;   //Max acceleration in sensor_units/100ms per second

        public static final int elevatorSCurveStrength = 3;

         /*Elevator PID constants.*/
        //TODO: Tune PID values using Phoenix tuner.
        public static final double elevatorKp = 1.0;
        public static final double elevatorKi = 0.0;
        public static final double elevatorKd = 0.0;

        //1023 is the max output of the motor controller.
        //kF = (PercentOutput X 1023) / Speed(Encoder Units/100ms)
        public static final double elevatorKF = (1.0 * 1023) / (6380 * (2048.0 / 600.0)); //TODO: Find actual speed at lower output percent for free motor.

        /*Elevator Gear Ratio*/
        public static final double elevatorGearRatio = 9.0 * 72.0 /22.0; //Motor revolutions to output shaft revolutions.

        //Total travel / number revolutions. 
        public static final double elevatorTravelEncoderTick = 16.5 / (288761.0 - 13301.0);
        public static final double elevatorMaxTravel = 16.5;    //Distance between limit switches.
        public static final double safeZone = 0.75;
    }

    public static final class Intake{
        public static final int intakeMotorID = 0;
        public static final int intakeSolenoidFWD = 4;
        public static final int intakeSolenoidREV = 5;

        public static final double intakeSpeed = 0.5;
        public static final double outtakeSpeed = -0.5;
    }

    public static final class Turntable{
        public static final int turntableMotorID = 23;
    }
}