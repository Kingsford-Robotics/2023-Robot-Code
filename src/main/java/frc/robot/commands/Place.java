// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Place {
    Arm arm;
    Elevator elevator;
    RobotContainer robotContainer;

    public Place(RobotContainer robotContainer, Arm arm, Elevator elevator) {
        this.robotContainer = robotContainer;
        this.arm = arm;
        this.elevator = elevator;
    }

    private double getTargetHeight(int level, boolean isCone)
    {
        if (isCone)
        {
            switch(level)
            {
                case 0:
                    return 7;
                case 1:
                    return 10;
                case 2:
                    return 13;
                default:
                    return 0.0;
            }
        }

        else{
            switch(level)
            {
                case 0:
                    return 7;
                case 1:
                    return 10;
                case 2:
                    return 13;
                default:
                    return 0.0;
            }
        }
    }

    public SequentialCommandGroup getCommand()
    {
        List<CommandBase> commandList = new ArrayList<CommandBase>();
        SequentialCommandGroup group;

        commandList.add(
            new InstantCommand(() -> arm.retract(), arm)
        );

        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(getTargetHeight(robotContainer.getLevel(), robotContainer.getIsCone()), 0.2), elevator)
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 8.0)
        );

        commandList.add(
            new InstantCommand(() -> arm.setArmAngle(0.0, 0.2), arm)
        );

        commandList.add(
            new WaitUntilCommand(() -> arm.isArmToPosition())
        );
        
        if(robotContainer.getLevel() == 2)
        {
            commandList.add(
                new InstantCommand(() -> arm.extend(), arm)
            );
        }

        group = new SequentialCommandGroup(commandList.toArray(new CommandBase[commandList.size()]));
        return group;
    }
}