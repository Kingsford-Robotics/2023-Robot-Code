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

  //Arm Data
  private GenericEntry armEncoderAngle;
  private GenericEntry armCancoderAngle;
  private GenericEntry isExtended;
  private GenericEntry isOpen;

  //Elevator Data
  private GenericEntry elevatorHeight;
  private GenericEntry bottomLimitSwitch;
  private GenericEntry topLimitSwitch;

  public ArmElevator() {
    /*Initialize Subsystems*/
    arm = new Arm();
    elevator = new Elevator(RobotConstants.ElevatorConstants.elevatorPivotStartHeight);

    /*Setup Shuffleboard*/
    ArmElevatorTab = Shuffleboard.getTab("Arm and Elevator");
    
    //Arm Data
    armEncoderAngle = ArmElevatorTab.add("Arm Encoder Angle", 0).getEntry();
    armCancoderAngle = ArmElevatorTab.add("Arm CANCoder Angle", 0).getEntry();
    isExtended = ArmElevatorTab.add("Arm Extended", false).getEntry();
    isOpen = ArmElevatorTab.add("Arm Open", true).getEntry();

    //Elevator Data
    elevatorHeight = ArmElevatorTab.add("Elevator Height", 0).getEntry();
    bottomLimitSwitch = ArmElevatorTab.add("Bottom Limit Switch", false).getEntry();
    topLimitSwitch = ArmElevatorTab.add("Top Limit Switch", false).getEntry();
  }

  public void setArmAngle(double angle) {
    arm.setArmAngle(angle);
  }

  public boolean isArmToPosition() {
    return arm.isArmToPosition();
  }

  public void setArmPercent(double percent) {
    arm.setArmSpeedPercent(percent);
  }

  public void setElevatorHeight(double height) {
    elevator.setElevatorHeight(height);
  }

  public boolean isElevatorToPosition() {
    return elevator.isElevatorToPosition();
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
    /*Output Shuffleboard Data*/

    //Arm Data
    armEncoderAngle.setDouble(arm.getAngle().getDegrees());
    armCancoderAngle.setDouble(arm.getAngle().getDegrees());
    isExtended.setBoolean(arm.isExtended());
    isOpen.setBoolean(arm.isOpen());
    
    //Elevator Data
    elevatorHeight.setDouble(elevator.getElevatorPosition());
    bottomLimitSwitch.setBoolean(elevator.getBottomLimitSwitch());
    topLimitSwitch.setBoolean(elevator.getTopLimitSwitch());
  }
}