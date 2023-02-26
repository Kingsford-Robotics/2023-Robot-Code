// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {
  /*Creates new Elevator */
  private TalonFX elevatorMotor;
  
  //Wired normally open (true = pressed)
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  /*Shuffleboard Setup*/
  private ShuffleboardTab elevatorTab;
  
  private GenericEntry elevatorHeight;
  private GenericEntry elevatorEncoder;
  private GenericEntry elevatorSpeed;

  private GenericEntry elevatorTopLimitSwitch;
  private GenericEntry elevatorBottomLimitSwitch;

  private GenericEntry isToPosition;

  private GenericEntry targetEncoderPosition;

  public Elevator(double initialPosition) {
    elevatorMotor = new TalonFX(RobotConstants.ElevatorConstants.elevatorMotorID);
    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
    elevatorMotor.setInverted(true);
    //Set motor PID constants
    elevatorMotor.config_kP(0, RobotConstants.ElevatorConstants.elevatorKp);
    elevatorMotor.config_kI(0, RobotConstants.ElevatorConstants.elevatorKi);
    elevatorMotor.config_kD(0, RobotConstants.ElevatorConstants.elevatorKd);
    elevatorMotor.config_kF(0, RobotConstants.ElevatorConstants.elevatorKF);

    //Configure Motion Magic
    elevatorMotor.configMotionCruiseVelocity(RobotConstants.ElevatorConstants.elevatorCruiseVelocity);
    elevatorMotor.configMotionAcceleration(RobotConstants.ElevatorConstants.elevatorMaxAcceleration);
    
    //Acceleration smoothing
    elevatorMotor.configMotionSCurveStrength(RobotConstants.ElevatorConstants.elevatorSCurveStrength);

    topLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.elevatorTopLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.elevatorBottomLimitSwitchID);

    calibrateElevator(initialPosition);

    /*Shuffleboard Setup*/
    elevatorTab = Shuffleboard.getTab("Elevator");

    elevatorHeight = elevatorTab.add("Elevator Height", 0.0).getEntry();
    elevatorEncoder = elevatorTab.add("Elevator Encoder", 0.0).getEntry();
    elevatorSpeed = elevatorTab.add("Elevator Speed", 0.0).getEntry();

    elevatorTopLimitSwitch = elevatorTab.add("Top Limit Switch", false).getEntry();
    elevatorBottomLimitSwitch = elevatorTab.add("Bottom Limit Switch", false).getEntry();

    isToPosition = elevatorTab.add("Is To Position", false).getEntry();
  }

  public void calibrateElevator(double currentHeight)
  {
    double encoderPosition = currentHeight * RobotConstants.ElevatorConstants.elevatorTravelEncoderTick;
    elevatorMotor.setSelectedSensorPosition(encoderPosition);
  }

  public void setElevatorSpeed(double speed){
    //Set speed to 0 if touching limit switch and moving towards it.
    if(getTopLimitSwitch() && speed > 0)
    {
      speed = 0;
    }
    else if(getBottomLimitSwitch() && speed < 0)
    {
      speed = 0;
    }

    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  //Set elevator height in meters relative to lowest position.
  public void setElevatorHeight(double height){
    double encoderPosition = height / RobotConstants.ElevatorConstants.elevatorTravelEncoderTick;

    targetEncoderPosition.setDouble(encoderPosition);

    elevatorMotor.set(ControlMode.MotionMagic, encoderPosition);
  }

  public boolean isElevatorToPosition() {
    if(elevatorMotor.getClosedLoopError(0) < 1.0 / RobotConstants.ElevatorConstants.elevatorTravelEncoderTick)
    {
      return true;
    }

    return false;
  }

  public boolean getTopLimitSwitch(){
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch(){
    return bottomLimitSwitch.get();
  }

  //Returns elevator height in meters relative to lowest position.
  public double getElevatorPosition(){
    return elevatorMotor.getSelectedSensorPosition() * RobotConstants.ElevatorConstants.elevatorTravelEncoderTick;
  } 

  public double getElevatorEncoder()
  {
    return elevatorMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    if(getTopLimitSwitch())
    {
      calibrateElevator(RobotConstants.ElevatorConstants.elevatorMaxTravel);

      if(elevatorMotor.getMotorOutputPercent() > 0)
      {
        elevatorMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    if(getBottomLimitSwitch())
    {
      calibrateElevator(0.0);

      if(elevatorMotor.getMotorOutputPercent() < 0)
      {
        elevatorMotor.set(ControlMode.PercentOutput, 0);
      }
    }

    elevatorHeight.setDouble(getElevatorPosition());
    elevatorEncoder.setDouble(getElevatorEncoder());
    elevatorSpeed.setDouble(elevatorMotor.getMotorOutputPercent());

    elevatorTopLimitSwitch.setBoolean(getTopLimitSwitch());
    elevatorBottomLimitSwitch.setBoolean(getBottomLimitSwitch());

    isToPosition.setBoolean(isElevatorToPosition());
  }
}