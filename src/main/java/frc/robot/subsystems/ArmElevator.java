// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElevator extends SubsystemBase {
  /** Creates a new ArmElevator. */
  private Arm arm;
  private Elevator elevator;

  private ShuffleboardTab ArmElevatorTab;
  private GenericEntry armEncoderAngle;
  private GenericEntry armCANCoderAngle;
  private GenericEntry armXY;
  private GenericEntry elevatorHeight;


  public ArmElevator() {
    arm = new Arm();
    elevator = new Elevator();
    ArmElevatorTab = Shuffleboard.getTab("Arm/Elevator");

    armEncoderAngle = ArmElevatorTab.add("Arm Encoder Angle", 0).getEntry();
    armCANCoderAngle = ArmElevatorTab.add("Arm CANCoder Angle", 0).getEntry();
    armXY = ArmElevatorTab.add("Arm XY", new int[]{0,0}).getEntry();
    elevatorHeight = ArmElevatorTab.add("Elevator Height", 0).getEntry();
  }

  @Override
  public void periodic() {
    armEncoderAngle.setDouble(arm.getAngle().getDegrees());
    armCANCoderAngle.setDouble(arm.getAngle().getDegrees());
    armXY.setDoubleArray(arm.getArmXY(elevator.getElevatorPosition()));
    elevatorHeight.setDouble(elevator.getElevatorPosition());
  }
}
