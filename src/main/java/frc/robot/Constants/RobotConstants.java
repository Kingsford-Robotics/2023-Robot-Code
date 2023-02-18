// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public final class RobotConstants {
    /*Sensors*/
    public static final int pigeonID = 1;
    
    //TODO: Find all device IDs.
    public static final int ArmEncoderID = 0;
    public static final double ArmEncoderOffset = 0.0;
    
    /*Motor IDs*/
    public static final int armMotorID = 0;
    public static final int intakeMotorID = 0;
    public static final int turntableMotorID = 0;
    public static final int elevatorMotorID = 0;

    /*Pneumatics*/
    public static final int armExtensionSolenoidFWD = 1;
    public static final int armExtensionSolenoidREV = 2;

    public static final int armGrabSolenoidFWD = 3;
    public static final int armGrabSolenoidREV = 4;

    /*Arm Constants*/

    //Configure arm motion magic constants.

    //TODO: Tune these values.
    //Values are set off of theoretical free speed RPM.
    //Free speed is 6380 RPM.
    
    private static final double percentSpeed = 0.5; //Percent of max speed to cruise at.
    public static final double armCruiseVelocity = 6380.0 * (2480.0 / 600.0) * percentSpeed; //Cruise velocity in sensor units per 100ms 
    
    private static final double timeToMaxSpeed = 0.5; //Time to reach cruise velocity in seconds
    public static final double armMaxAcceleration = armCruiseVelocity * 1.0 / timeToMaxSpeed;   //Max acceleration in sensor_units/100ms per second

    public static final int sCurveStrength = 3;
    
    //Configure arm gravity compensation
    public static final double armMaxGravityComp = 0.0;     //TODO: Find gravity compensation coefficient.

    //Configure arm PID constants

    //TODO: Tune PID values using Phoenix tuner.
    public static final double armKp = 0.0;
    public static final double armKi = 0.0;
    public static final double armKd = 0.0;


    //1023 is the max output of the motor controller.
    //6380 RPM is the free speed of the arm motor.
    //kF = (PercentOutput X 1023) / Speed(Encoder Units/100ms)
    public static final double armKF = (1.0 * 1023) / (6380 * (2048.0 / 600.0));    //TODO: Find actual speed at lower output percent for free motor.

    /*Arm Gear Ratio*/
    public static final double armGearRatio = 9.0 * 72.0 /22.0; //Motor revolutions to output shaft revolutions.
}