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
    /** Creates a new Arm. */
    private TalonFX armMotor;
    private CANCoder angleEncoder;

    private DoubleSolenoid armExtension;
    private DoubleSolenoid armGrab;

    public Arm() {
        /*Arm Motor Setup*/
        armMotor = new TalonFX(RobotConstants.Arm.armMotorID);
        armMotor.configFactoryDefault();
        armMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        
        //Set motor PID constants
        armMotor.config_kP(0, RobotConstants.Arm.armKp);
        armMotor.config_kI(0, RobotConstants.Arm.armKi);
        armMotor.config_kD(0, RobotConstants.Arm.armKd);
        armMotor.config_kF(0, RobotConstants.Arm.armKF);

        //Configure Motion Magic
        armMotor.configMotionCruiseVelocity(RobotConstants.Arm.armCruiseVelocity);
        armMotor.configMotionAcceleration(RobotConstants.Arm.armMaxAcceleration);
        
        //Acceleration smoothing
        armMotor.configMotionSCurveStrength(RobotConstants.Arm.armSCurveStrength);

        /*Arm Encoder Setup*/
        angleEncoder = new CANCoder(RobotConstants.Arm.armEncoderID);
        angleEncoder.configFactoryDefault();
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        angleEncoder.configAllSettings(encoderConfig);

        /*Pneumatics Setup*/
        armExtension = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.Arm.armExtensionSolenoidFWD, 
            RobotConstants.Arm.armExtensionSolenoidREV
        );

        armGrab = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.Arm.armGrabSolenoidFWD, 
            RobotConstants.Arm.armGrabSolenoidREV
        );

        //Set default arm position
        armExtension.set(kReverse); //Arm retracted
        armGrab.set(kForward);      //Claw open

        //Reset encoder to absolute position
        resetToAbsolute();      //TODO: Check if wait is needed. May be good from delay when initializing Swerve subsystem.
    }

    public void toggleGrab(){
        armGrab.toggle();
    }

    public void close(){
        armGrab.set(kForward);
    }

    public void open(){
        armGrab.set(kReverse);
    }

    public boolean isOpen(){
        return armGrab.get() == kReverse;
    }

    public void toggleExtension(){
        armExtension.toggle();
    }

    public void extend(){
        armExtension.set(kForward);
    }

    public void retract(){
        armExtension.set(kReverse);
    }

    public boolean isExtended(){
        return armExtension.get() == kForward;
    }

    public void setArmSpeedPercent(double speed){
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setArmPosition(double position){
        armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, RobotConstants.Arm.armMaxGravityComp * Math.cos(getAngle().getRadians()));
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - RobotConstants.Arm.armEncoderOffset, RobotConstants.Arm.armGearRatio);
        armMotor.setSelectedSensorPosition(absolutePosition);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(armMotor.getSelectedSensorPosition(), RobotConstants.Arm.armGearRatio));
    }

    public double getVelocity(){
        return Conversions.falconToRPM(armMotor.getSelectedSensorVelocity(), RobotConstants.Arm.armGearRatio);
    }

    public double[] getArmXY()
    {
        double[] armXY = new double[2];
        armXY[0] = Math.cos(getAngle().getRadians());
        armXY[1] = Math.sin(getAngle().getRadians());
        return armXY;
    }
    
    public static boolean isCollision(double elevatorHeight)
    {
        
        return true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        //Stop arm if outside range
        if(getAngle().getDegrees() > RobotConstants.Arm.armMaxAngle || getAngle().getDegrees() < RobotConstants.Arm.armMinAngle){
            armMotor.set(ControlMode.PercentOutput, 0);
        }
    }
}