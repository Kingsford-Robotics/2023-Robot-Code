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
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Arm extends SubsystemBase {
    /** Creates a new Arm. */
    private TalonFX armMotor;
    private CANCoder angleEncoder;

    private DoubleSolenoid armExtension;
    private DoubleSolenoid armGrab;

    /*Shuffleboard Data*/
    private ShuffleboardTab armTab;

    private GenericEntry encoderAngle;
    private GenericEntry cancoderAngle;

    private GenericEntry extensionState;
    private GenericEntry grabState;

    private GenericEntry armMotorOutput;

    private GenericEntry isArmToPosition;
    private GenericEntry closedLoopError;

    public Arm() {
        /*Arm Motor Setup*/
        armMotor = new TalonFX(RobotConstants.ArmConstants.armMotorID);
        armMotor.configFactoryDefault();
        armMotor.setInverted(true);

        armMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        
        //Set motor PID constants
        armMotor.config_kP(0, RobotConstants.ArmConstants.armKp);
        armMotor.config_kI(0, RobotConstants.ArmConstants.armKi);
        armMotor.config_kD(0, RobotConstants.ArmConstants.armKd);
        armMotor.config_kF(0, RobotConstants.ArmConstants.armKF);

        //Configure Motion Magic
        armMotor.configMotionCruiseVelocity(RobotConstants.ArmConstants.armCruiseVelocity);
        armMotor.configMotionAcceleration(RobotConstants.ArmConstants.armMaxAcceleration);
        
        //Acceleration smoothing
        armMotor.configMotionSCurveStrength(RobotConstants.ArmConstants.armSCurveStrength);

        /*Arm Encoder Setup*/
        angleEncoder = new CANCoder(RobotConstants.ArmConstants.armEncoderID);
        angleEncoder.configFactoryDefault();

        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.sensorDirection = true;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        
        angleEncoder.configAllSettings(encoderConfig);

        /*Pneumatics Setup*/
        armExtension = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.ArmConstants.armExtensionSolenoidFWD, 
            RobotConstants.ArmConstants.armExtensionSolenoidREV
        );

        armGrab = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.ArmConstants.armGrabSolenoidFWD, 
            RobotConstants.ArmConstants.armGrabSolenoidREV
        );

        //Set default arm position
        armExtension.set(kReverse); //Arm retracted
        armGrab.set(kForward);      //Claw open

        /*Shuffleboard Setup*/
        armTab = Shuffleboard.getTab("Arm");

        encoderAngle = armTab.add("Encoder Angle", 0).getEntry();
        cancoderAngle = armTab.add("CANCoder Angle", 0).getEntry();

        extensionState = armTab.add("Extension State", "Retracted").getEntry();
        grabState = armTab.add("Grab State", "Open").getEntry();

        armMotorOutput = armTab.add("Arm Motor Output", 0).getEntry();
        isArmToPosition = armTab.add("Is Arm to Position", false).getEntry();
        closedLoopError = armTab.add("Arm Closed Loop Error", 0).getEntry();

        /*Reset to Absolute */
        Timer.delay(0.5);   //Check to see if this is needed. May be good from delay when initializing Swerve subsystem.
        //Reset encoder to absolute position
        resetToAbsolute();
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

    public void setArmSpeed(double speed){
        //Stop motor if beyond limit and moving into the restricted area.
        if(armMotor.getSelectedSensorPosition() >= RobotConstants.ArmConstants.armMaxAngle && speed > 0){
            speed = 0;
        }

        else if(armMotor.getSelectedSensorPosition() <= RobotConstants.ArmConstants.armMinAngle && speed < 0){
            speed = 0;
        }

        armMotor.set(ControlMode.PercentOutput, speed);
    }

    //Set arm angle in degrees.
    public void setArmAngle(double angle){
        //Check if angle is within range.
        if(angle > RobotConstants.ArmConstants.armMaxAngle){
            angle = RobotConstants.ArmConstants.armMaxAngle;
        }

        else if(angle < RobotConstants.ArmConstants.armMinAngle){
            angle = RobotConstants.ArmConstants.armMinAngle;
        }

        double position = Conversions.degreesToFalcon(angle, RobotConstants.ArmConstants.armGearRatio);
        armMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, Math.cos(getAngle().getRadians()) * RobotConstants.ArmConstants.armMaxGravityComp);
    }

    //TODO: Check if function works.
    public boolean isArmToPosition()
    {
       if(armMotor.getClosedLoopError(0) < Conversions.degreesToFalcon(2, RobotConstants.ElevatorConstants.elevatorTravelEncoderTick))
       {
        return true;
       }

       return false;
    }

    public void resetToAbsolute(){
        double angle;
        if(getCanCoder().getDegrees() - RobotConstants.ArmConstants.armEncoderOffset <= -180)
        {
            angle = 360 + getCanCoder().getDegrees() - RobotConstants.ArmConstants.armEncoderOffset;
        }

        else
        {
            angle = getCanCoder().getDegrees() - RobotConstants.ArmConstants.armEncoderOffset;
        }

        double encoderTicks = Conversions.degreesToFalcon(angle, RobotConstants.ArmConstants.armGearRatio);
        armMotor.setSelectedSensorPosition((int)encoderTicks);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(armMotor.getSelectedSensorPosition(), RobotConstants.ArmConstants.armGearRatio));
    }

    public double getRPM(){
        return Conversions.falconToRPM(armMotor.getSelectedSensorVelocity(), RobotConstants.ArmConstants.armGearRatio);
    }

    @Override
    public void periodic() {
        if(armMotor.getSelectedSensorPosition() >= RobotConstants.ArmConstants.armMaxAngle && armMotor.getMotorOutputPercent() > 0){
            armMotor.set(ControlMode.PercentOutput, 0);
        }

        if(armMotor.getSelectedSensorPosition() <= RobotConstants.ArmConstants.armMinAngle && armMotor.getMotorOutputPercent() < 0){
            armMotor.set(ControlMode.PercentOutput, 0);
        }

        //Update Shuffleboard
        encoderAngle.setDouble(getAngle().getDegrees());
        cancoderAngle.setDouble(getCanCoder().getDegrees());

        extensionState.setString(isExtended() ? "Extended" : "Retracted");
        grabState.setString(isOpen() ? "Open" : "Closed");

        armMotorOutput.setDouble(armMotor.getMotorOutputPercent());
        isArmToPosition.setBoolean(isArmToPosition());
        closedLoopError.setDouble(armMotor.getClosedLoopError(0));
    }
}