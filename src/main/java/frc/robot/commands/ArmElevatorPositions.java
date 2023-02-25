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
  
  //Create positions enum for cone and cube positions. TOP, MIDDLE, FLOOR

  public enum Positions{CONE_TOP, CONE_MIDDLE, CONE_FLOOR, CUBE_TOP, CUBE_MIDDLE, CUBE_FLOOR};
  
  private ArmElevator m_ArmElevator;
  
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
          new ParallelCommandGroup(
            new InstantCommand(() -> m_ArmElevator.setElevatorHeight(3))
          ),

          new WaitUntilCommand(() -> m_ArmElevator.isElevatorToPosition()),

          new ParallelCommandGroup(
            new InstantCommand(() -> m_ArmElevator.setArmAngle(0)),
            new InstantCommand(() -> m_ArmElevator.setElevatorHeight(5))
          ),

          new WaitUntilCommand(() -> m_ArmElevator.isArmToPosition()),
          new WaitUntilCommand(() -> m_ArmElevator.isElevatorToPosition())
        );

      case CONE_MIDDLE:
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_ArmElevator.setArmAngle(0)),
          new InstantCommand(() -> m_ArmElevator.setElevatorHeight(0))
        );

      case CONE_FLOOR:
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_ArmElevator.setArmAngle(0)),
          new InstantCommand(() -> m_ArmElevator.setElevatorHeight(0))
        );

      case CUBE_TOP:
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_ArmElevator.setArmAngle(0)),
          new InstantCommand(() -> m_ArmElevator.setElevatorHeight(0))
        );

      case CUBE_MIDDLE:
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_ArmElevator.setArmAngle(0)),
          new InstantCommand(() -> m_ArmElevator.setElevatorHeight(0))
        );

      case CUBE_FLOOR:
        return new SequentialCommandGroup(
          new InstantCommand(() -> m_ArmElevator.setArmAngle(0)),
          new InstantCommand(() -> m_ArmElevator.setElevatorHeight(0))
        );
    }
    return null;
  }
}
