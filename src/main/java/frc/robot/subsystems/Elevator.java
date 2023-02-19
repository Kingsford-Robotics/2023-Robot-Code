// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
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

  public void setElevatorSpeed(double speed){
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  //TODO: Call at the beginning of autonomous. Ensures elevator is calibrated with the limit switches.
  public SequentialCommandGroup resetToAbsolute(){
    //TODO: Not done; fix sequential command group structure. Look at examples of using lambdas.  
    return new SequentialCommandGroup(
        new Command(){
          @Override
          public void initialize() {
            elevatorMotor.setSelectedSensorPosition(0);
          }
          @Override
          public boolean isFinished() {
            return true;
          }
          @Override
          public Set<Subsystem> getRequirements() {
            // TODO Auto-generated method stub
            return null;
          }
        },
        new Command(){
          @Override
          public void initialize() {
            elevatorMotor.set(ControlMode.MotionMagic, 0);
          }
          @Override
          public boolean isFinished() {
            return true;
          }

          @Override
          public Set<Subsystem> getRequirements() {
            // TODO Auto-generated method stub
            return null;
          }
        }
      );
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}