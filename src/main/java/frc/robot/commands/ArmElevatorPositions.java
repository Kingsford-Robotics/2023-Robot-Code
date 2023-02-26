// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ArmElevatorPositions{

  private final double clearanceHeight = 5.0;

  public enum Positions {
    CONE_TOP,
    CONE_MIDDLE,
    CONE_FLOOR,
    CUBE_TOP,
    CUBE_MIDDLE,
    CUBE_FLOOR,
    HOME,
    STARTING,
    GROUND_PICKUP,
    TURNTABLE_PICKUP,
  };

  private Arm arm;
  private Elevator elevator;

  public ArmElevatorPositions(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public SequentialCommandGroup getArmElevatorCommand(Positions position) {
    switch (position) {
      case CONE_TOP:
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
            new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
            new InstantCommand(() -> arm.setArmAngle(0.0)),
            new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
            new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
            new WaitUntilCommand(() -> arm.isArmToPosition()));

      case CONE_MIDDLE:
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
            new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
            new InstantCommand(() -> arm.setArmAngle(0.0)),
            new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
            new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
            new WaitUntilCommand(() -> arm.isArmToPosition()));

      case CONE_FLOOR:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case CUBE_TOP:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case CUBE_MIDDLE:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case CUBE_FLOOR:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case HOME:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case STARTING:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case GROUND_PICKUP:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      case TURNTABLE_PICKUP:
      return new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new InstantCommand(() -> arm.setArmAngle(0.0)),
        new InstantCommand(() -> elevator.setElevatorHeight(10.0)),
        new WaitUntilCommand(() -> elevator.isElevatorToPosition()),
        new WaitUntilCommand(() -> arm.isArmToPosition())
      );

      default:
        return new SequentialCommandGroup();
    }
  }
}