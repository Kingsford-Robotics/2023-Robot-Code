// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public final class RobotConstants {
    /*Sensors*/
    public static final int pigeonID = 1;

    public final class Arm {
        public static final int armMotorID = 0;
        
        /*Arm Pneumatics*/
        public static final int armExtensionSolenoidFWD = 1;
        public static final int armExtensionSolenoidREV = 2;

        public static final int armGrabSolenoidFWD = 3;
        public static final int armGrabSolenoidREV = 4;

        /*Arm Encoder Values*/
        public static final int armEncoderID = 0;
        public static final double armEncoderOffset = 0.0;  //TODO: Set to cancoder angle in degrees at horizontal.

        /*Arm Motion Magic Constants*/
        private static final double armMaxSpeed = 6380.0 * (2480.0 / 600.0); //Max speed in sensor units per 100ms
        private static final double armPercentSpeed = 0.5; //Percent of max speed to cruise at.

        public static final double armCruiseVelocity = armMaxSpeed * armPercentSpeed; //Cruise velocity in sensor units per 100ms 
    
        private static final double armTimeToMaxSpeed = 0.5; //Time to reach cruise velocity in seconds
        public static final double armMaxAcceleration = armCruiseVelocity * 1.0 / armTimeToMaxSpeed;   //Max acceleration in sensor units/100ms per second

        public static final int armSCurveStrength = 3;

        /*Arm Gravity Compensation*/
        //TODO: Set to minimum percent output needed to hold arm up at horizontal.
        public static final double armMaxGravityComp = 0.15;    //Percent output

        /*Arm PID Constants*/
        //TODO: Tune PID values using Phoenix tuner.
        public static final double armKp = 0.0;
        public static final double armKi = 0.0;
        public static final double armKd = 0.0;

        //1023 is the max output of the motor controller.
        //kF = (PercentOutput X 1023) / Speed(Encoder Units/100ms)
        public static final double armKF = 1.0 * 1023.0 / armMaxSpeed;    //TODO: Find actual speed at lower output percent for free motor.

        /*Arm Gear Ratio*/
        public static final double armGearRatio = 9.0 * 72.0 /22.0; //Motor revolutions to output shaft revolutions.
    }
  
    public final class Elevator{
        public static final int elevatorMotorID = 0;

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

        /*Elevator Gravity Compensation*/
        //TODO: Set to minimum percent output needed to hold elevator up.
        public static final double elevatorGravityComp = 0.15;    //Percent output

         /*Elevator PID constants.*/
        //TODO: Tune PID values using Phoenix tuner.
        public static final double elevatorKp = 0.0;
        public static final double elevatorKi = 0.0;
        public static final double elevatorKd = 0.0;

        //1023 is the max output of the motor controller.
        //kF = (PercentOutput X 1023) / Speed(Encoder Units/100ms)
        public static final double elevatorKF = (1.0 * 1023) / (6380 * (2048.0 / 600.0)); //TODO: Find actual speed at lower output percent for free motor.

        /*Elevator Gear Ratio*/
        public static final double elevatorGearRatio = 9.0 * 72.0 /22.0; //Motor revolutions to output shaft revolutions.
    }

    public final class Intake{
        public static final int intakeMotorID = 0;
    }

    public final class Turntable{
        public static final int turntableMotorID = 0;
    }
}