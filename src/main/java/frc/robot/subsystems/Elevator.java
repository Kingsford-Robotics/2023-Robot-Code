// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    elevatorMotor = new TalonFX(0);
    elevatorMotor.configFactoryDefault();
    elevatorMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);

    //Set motor PID constants
    elevatorMotor.config_kP(0, RobotConstants.elevatorKp);
    elevatorMotor.config_kI(0, RobotConstants.elevatorKi);
    elevatorMotor.config_kD(0, RobotConstants.elevatorKd);
    elevatorMotor.config_kF(0, RobotConstants.elevatorKF);

    //Configure Motion Magic
    elevatorMotor.configMotionCruiseVelocity((int)(RobotConstants.elevatorCruiseVelocity * RobotConstants.elevatorGearRatio));
    elevatorMotor.configMotionAcceleration((int)(RobotConstants.elevatorMaxAcceleration * RobotConstants.elevatorGearRatio));
    
    //Acceleration smoothing
    elevatorMotor.configMotionSCurveStrength(RobotConstants.elevatorSCurveStrength);

    topLimitSwitch = new DigitalInput(RobotConstants.elevatorTopLimitSwitchID);
    bottomLimitSwitch = new DigitalInput(RobotConstants.elevatorBottomLimitSwitchID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}