// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmElevator;

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
  
  private ArmElevator m_ArmElevator;

  private final SequentialCommandGroup elevatorClearance = new SequentialCommandGroup(
    new InstantCommand(() -> m_ArmElevator.setElevatorHeight(clearanceHeight)),
    new WaitUntilCommand(() -> m_ArmElevator.isElevatorToPosition())
  );

  private final SequentialCommandGroup waitToPosition = new SequentialCommandGroup(
    new WaitUntilCommand(() -> m_ArmElevator.isArmToPosition()),
    new WaitUntilCommand(() -> m_ArmElevator.isElevatorToPosition())
  );

  private final SequentialCommandGroup setArmElevatorPosition(double armAngle, double elevatorHeight)
  {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new InstantCommand(() -> m_ArmElevator.setArmAngle(armAngle)),
        new InstantCommand(() -> m_ArmElevator.setElevatorHeight(elevatorHeight))
      ),
      waitToPosition
    );
  }
  
  public ArmElevatorPositions(ArmElevator armElevator)
  {
    m_ArmElevator = armElevator;
  }

  public SequentialCommandGroup getArmElevatorCommand(Positions position)
  {
    switch(position)
    {
      case CONE_TOP:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );

      case CONE_MIDDLE:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
        );

      case CONE_FLOOR:
        return new SequentialCommandGroup(
          elevatorClearance,
          setArmElevatorPosition(0, 0)
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
          new InstantCommand(() -> m_ArmElevator.openClaw()),
          setArmElevatorPosition(0, 0)
        );

      case TURNTABLE_PICKUP:
        return new SequentialCommandGroup(
          elevatorClearance,
          new InstantCommand(() -> m_ArmElevator.openClaw()),
          setArmElevatorPosition(0, 0)
        );

      default:
        return new SequentialCommandGroup();
    }
  }
}