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

  private ShuffleboardTab ArmElevatorTab;
  private GenericEntry armEncoderAngle;
  private GenericEntry armCANCoderAngle;
  private GenericEntry armXY;
  private GenericEntry elevatorHeight;
  private GenericEntry bottomLimitSwitch;
  private GenericEntry topLimitSwitch;
  private GenericEntry elevatorEncoder;


  public ArmElevator() {
    arm = new Arm();
    elevator = new Elevator(0.0);   //TODO: Set initial position
    ArmElevatorTab = Shuffleboard.getTab("Arm_Elevator");

    armEncoderAngle = ArmElevatorTab.add("Arm Encoder Angle", 0).getEntry();
    armCANCoderAngle = ArmElevatorTab.add("Arm CANCoder Angle", 0).getEntry();
    armXY = ArmElevatorTab.add("Arm XY", "").getEntry();
    elevatorHeight = ArmElevatorTab.add("Elevator Height", 0).getEntry();
    bottomLimitSwitch = ArmElevatorTab.add("Bottom Limit Switch", false).getEntry();
    topLimitSwitch = ArmElevatorTab.add("Top Limit Switch", false).getEntry();
    elevatorEncoder = ArmElevatorTab.add("Elevator Encoder", 0.0).getEntry();
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
    if (percent < 0 && (elevator.getBottomLimitSwitch() || elevator.getElevatorPosition() <= RobotConstants.ElevatorConstants.elevatorMinHeight))
    {
      percent = 0;
    }

    else if (percent > 0 && (elevator.getTopLimitSwitch() || elevator.getElevatorPosition() >= RobotConstants.ElevatorConstants.elevatorMaxHeight))
    {
      percent = 0;
    }

    if (isCollision(elevator.getElevatorPosition(), arm.getAngle().getDegrees(), arm.isExtended()))
    {
      percent = 0;
    }

    else if (isCollision(elevator.getElevatorPosition(), arm.getAngle().getDegrees() + 3, arm.isExtended()))
    {
      percent = 0;
    }

    else if (isCollision(elevator.getElevatorPosition(), arm.getAngle().getDegrees() - 3, arm.isExtended()))
    {
      percent = 0;
    }

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
    if(!isCollision(elevator.getElevatorPosition(), arm.getAngle().getDegrees(), true))
    {
      arm.extend();
    }
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

  private double getArmMinY(double armX)
  {
    if (armX < RobotConstants.HeightZones.turntableEndX)
    {
      return RobotConstants.HeightZones.turntable;
    }
    else if (armX < RobotConstants.HeightZones.midBotEndX)
    {
      return RobotConstants.HeightZones.midBot;
    }
    else
    {
      return RobotConstants.HeightZones.outside;
    }
  }

  private boolean isCollision(double elevatorHeight, double armAngle, boolean isExtended)
  {
    double armX = arm.getArmXY(elevator.getElevatorPosition(), armAngle, isExtended)[0];
    double armY = arm.getArmXY(elevator.getElevatorPosition(), armAngle, isExtended)[1];
    double minY = getArmMinY(armX);

    if (armY < minY)
    {
      return true;
    }

    return false;
  }
  @Override
  public void periodic() {
    armEncoderAngle.setDouble(arm.getAngle().getDegrees());
    armCANCoderAngle.setDouble(arm.getCanCoder().getDegrees());
    elevatorHeight.setDouble(elevator.getElevatorPosition());
    armXY.setString(String.valueOf(arm.getArmXY(elevator.getElevatorPosition())[0]) + ", " + String.valueOf(arm.getArmXY(elevator.getElevatorPosition())[1]));
    bottomLimitSwitch.setBoolean(elevator.getBottomLimitSwitch());
    topLimitSwitch.setBoolean(elevator.getTopLimitSwitch());
    elevatorEncoder.setDouble(elevator.getElevatorEncoder());
  }
}