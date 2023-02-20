// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Elevator extends SubsystemBase {
  /*Creates new Elevator */
  private TalonFX elevatorMotor;
  
  //Wired normally open (true = pressed)
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  
  public Elevator() {
    elevatorMotor = new TalonFX(RobotConstants.Elevator.elevatorMotorID);
    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

    //Set motor PID constants
    elevatorMotor.config_kP(0, RobotConstants.Elevator.elevatorKp);
    elevatorMotor.config_kI(0, RobotConstants.Elevator.elevatorKi);
    elevatorMotor.config_kD(0, RobotConstants.Elevator.elevatorKd);
    elevatorMotor.config_kF(0, RobotConstants.Elevator.elevatorKF);

    //Configure Motion Magic
    elevatorMotor.configMotionCruiseVelocity(RobotConstants.Elevator.elevatorCruiseVelocity);
    elevatorMotor.configMotionAcceleration(RobotConstants.Elevator.elevatorMaxAcceleration);
    
    //Acceleration smoothing
    elevatorMotor.configMotionSCurveStrength(RobotConstants.Elevator.elevatorSCurveStrength);

    topLimitSwitch = new DigitalInput(RobotConstants.Elevator.elevatorTopLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(RobotConstants.Elevator.elevatorBottomLimitSwitchID);
  }

  public void setElevatorSpeed(double speed){
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  //Set elevator height in meters from lowest position.
  public void setElevatorPosition(double position){
    double encoderPosition = position / RobotConstants.Elevator.elevatorTravelPerRev * 2048.0;
    elevatorMotor.set(ControlMode.MotionMagic, encoderPosition, DemandType.ArbitraryFeedForward, RobotConstants.Elevator.elevatorGravityComp);
  }

  public boolean getTopLimitSwitch(){
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch(){
    return bottomLimitSwitch.get();
  }

  //Returns elevator position in meters from lowest position.
  public double getElevatorPosition(){
    return elevatorMotor.getSelectedSensorPosition() / 2048.0 * RobotConstants.Elevator.elevatorTravelPerRev;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Stop motor if moving up and top limit switch pressed
    if(elevatorMotor.getMotorOutputPercent() > 0 && getTopLimitSwitch()){
      elevatorMotor.set(ControlMode.PercentOutput, 0);
      
    }

    //Stop motor if moving down and bottom limit switch pressed
    if(elevatorMotor.getMotorOutputPercent() < 0 && getBottomLimitSwitch()){
      elevatorMotor.set(ControlMode.PercentOutput, 0);
    }
  }
}