// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ArmElevatorPositions extends CommandBase {
  
  private final double clearanceHeight = 5.0;

  public enum Positions{
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

  private final SequentialCommandGroup elevatorClearance = new SequentialCommandGroup(
    new InstantCommand(() -> elevator.setElevatorHeight(clearanceHeight)),
    new WaitUntilCommand(() -> elevator.isElevatorToPosition())
  );

  private final SequentialCommandGroup waitToPosition = new SequentialCommandGroup(
    new WaitUntilCommand(() -> arm.isArmToPosition()),
    new WaitUntilCommand(() -> elevator.isElevatorToPosition())
  );

  private final SequentialCommandGroup setArmElevatorPosition(double armAngle, double elevatorHeight)
  {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new InstantCommand(() -> arm.setArmAngle(armAngle)),
        new InstantCommand(() -> elevator.setElevatorHeight(elevatorHeight))
      ),
      waitToPosition
    );
  }
  
  public ArmElevatorPositions(Arm arm, Elevator elevator)
  {
    this.arm = arm;
    this.elevator = elevator;
  }

  public SequentialCommandGroup getArmElevatorCommand(Positions position)
  {
    switch(position)
    {
      case CONE_TOP:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0.0, 10.0)
        );

      case CONE_MIDDLE:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0.0, 5.0)
        );

      case CONE_FLOOR:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(45, 2.5)
        );
      
      case CUBE_TOP:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );

      case CUBE_MIDDLE:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );

      case CUBE_FLOOR:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );
      
      case HOME:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );

      case STARTING:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );

      case GROUND_PICKUP:
        return new SequentialCommandGroup(
          elevatorClearance,
          new InstantCommand(() -> arm.open()),
          setArmElevatorPosition(0, 0)
        );

      case TURNTABLE_PICKUP:
        return new SequentialCommandGroup(
          elevatorClearance,
          new InstantCommand(() -> arm.open()),
          setArmElevatorPosition(0, 0)
        );

      default:
        return new SequentialCommandGroup();
    }
  }
}