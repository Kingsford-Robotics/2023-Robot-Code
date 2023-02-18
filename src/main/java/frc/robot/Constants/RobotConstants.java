// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public final class RobotConstants {
    /*Sensors*/
    public static final int pigeonID = 1;

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
    public static final double armMaxVelocity = 0.5;   //Max speed in radians/s
    public static final double armMaxAcceleration = 0.5;   //Max acceleration in radians/s^2

    //Configure arm gravity compensation
    public static final double armMaxGravityComp = 0.0;

    //Configure arm PID constants
    public static final double armKp = 0.0;
    public static final double armKi = 0.0;
    public static final double armKd = 0.0;

    /*Arm Gear Ratio*/
    public static final double armGearRatio = 1.0;
}
