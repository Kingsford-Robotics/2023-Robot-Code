// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Arm extends SubsystemBase {
    /** Creates a new Elevator. */
    private TalonFX armMotor;
    private CANCoder angleEncoder;

    private DoubleSolenoid armExtension;
    private DoubleSolenoid armGrab;

    public Arm() {
        /*Arm Motor Setup*/
        armMotor = new TalonFX(RobotConstants.armMotorID);
        armMotor.configFactoryDefault();
        armMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        
        //Set motor PID constants
        armMotor.config_kP(0, RobotConstants.armKp);
        armMotor.config_kI(0, RobotConstants.armKi);
        armMotor.config_kD(0, RobotConstants.armKd);
        armMotor.config_kF(0, RobotConstants.armKF);

        //Configure Motion Magic
        armMotor.configMotionCruiseVelocity((int)(RobotConstants.armCruiseVelocity * RobotConstants.armGearRatio));
        armMotor.configMotionAcceleration((int)(RobotConstants.armMaxAcceleration * RobotConstants.armGearRatio));
        
        //Acceleration smoothing
        armMotor.configMotionSCurveStrength(RobotConstants.sCurveStrength);

        /*Arm Encoder Setup*/
        angleEncoder = new CANCoder(RobotConstants.ArmEncoderID);
        angleEncoder.configFactoryDefault();
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        angleEncoder.configAllSettings(encoderConfig);

        /*Pneumatics Setup*/
        armExtension = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.armExtensionSolenoidFWD, 
            RobotConstants.armExtensionSolenoidREV
        );

        armGrab = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.armGrabSolenoidFWD, 
            RobotConstants.armGrabSolenoidREV
        );

        //Set default arm position
        armExtension.set(kReverse); //Arm retracted
        armGrab.set(kForward);      //Claw open

        //Reset encoder to absolute position
        resetToAbsolute();      //TODO: Check if wait is needed. May be good from delay when initializing Swerve subsystem.
    }

    void toggleGrab(){
        armGrab.toggle();
    }

    void close(){
        armGrab.set(kForward);
    }

    void open(){
        armGrab.set(kReverse);
    }

    boolean isOpen(){
        return armGrab.get() == kReverse;
    }

    void toggleExtension(){
        armExtension.toggle();
    }

    void extend(){
        armExtension.set(kForward);
    }

    void retract(){
        armExtension.set(kReverse);
    }

    boolean isExtended(){
        return armExtension.get() == kForward;
    }

    void setArmSpeedPercent(double speed){
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    void setArmPosition(double position){
        armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, RobotConstants.armMaxGravityComp * Math.cos(getAngle().getRadians()));
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - RobotConstants.ArmEncoderOffset, RobotConstants.armGearRatio);
        armMotor.setSelectedSensorPosition(absolutePosition);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(armMotor.getSelectedSensorPosition(), RobotConstants.armGearRatio));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run\
    }
}