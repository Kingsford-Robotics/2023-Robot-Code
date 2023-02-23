// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ArmElevator extends SubsystemBase {
  /** Creates a new ArmElevator. */
  private Arm arm;
  private Elevator elevator;

  //Shuffleboard Data
  private ShuffleboardTab ArmElevatorTab;

  private GenericEntry armEncoderAngle;
  private GenericEntry bottomLimitSwitch;
  private GenericEntry topLimitSwitch;

  private GenericEntry elevatorHeight;

  public ArmElevator() {
    arm = new Arm();
    elevator = new Elevator(RobotConstants.ElevatorConstants.elevatorPivotStartHeight);   //TODO: Set initial position

    ArmElevatorTab = Shuffleboard.getTab("Arm_Elevator");
    //Setup Shuffleboard
    armEncoderAngle = ArmElevatorTab.add("Arm Encoder Angle", 0).getEntry();
    bottomLimitSwitch = ArmElevatorTab.add("Bottom Limit Switch", false).getEntry();
    topLimitSwitch = ArmElevatorTab.add("Top Limit Switch", false).getEntry();
    elevatorHeight = ArmElevatorTab.add("Elevator Height", 0).getEntry();
  }

  public void setArmAngle(double angle) {
    arm.setArmAngle(angle);
  }

  public void setArmPercent(double percent) {
    arm.setArmSpeedPercent(percent);
  }

  public void setElevatorHeight(double height) {
    elevator.setElevatorHeight(height);
  }

  public void setElevatorPercent(double percent)
  {
    elevator.setElevatorSpeed(percent);
  }

  public void toggleExtension() {
    if(arm.isExtended())
    {
      retractArm();
    }

    else{
      extendArm();
    }
  }

  public void extendArm() {
    arm.extend();
  }

  public void retractArm() {
    arm.retract();
  }

  public void closeClaw()
  {
    arm.close();
  }

  public void openClaw()
  {
    arm.open();
  }

  @Override
  public void periodic() {
    //Setup Shuffleboard
    armEncoderAngle.setDouble(arm.getAngle().getDegrees());
    bottomLimitSwitch.setBoolean(elevator.getBottomLimitSwitch());
    topLimitSwitch.setBoolean(elevator.getTopLimitSwitch());
    elevatorHeight.setDouble(elevator.getElevatorPosition());
  }
}