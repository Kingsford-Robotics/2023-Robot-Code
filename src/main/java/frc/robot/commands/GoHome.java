// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class GoHome {
    Arm arm;
    Elevator elevator;

    public GoHome(Arm arm, Elevator elevator) {
        this.arm = arm;
        this.elevator = elevator;
    }

    public SequentialCommandGroup getCommand() {
        List<CommandBase> commandList = new ArrayList<CommandBase>();

        SequentialCommandGroup group;

        commandList.add(
            new InstantCommand(() -> arm.retract()))
        ;

        commandList.add(
            new InstantCommand(() -> arm.open(), elevator)
        );
        
        if (elevator.getElevatorPosition() < 7.5)
        {
            commandList.add(
                new InstantCommand(() -> elevator.setElevatorHeight(7.5, 0.7), elevator)
            );

            commandList.add(
                new WaitUntilCommand(() -> elevator.isElevatorToPosition())
            );
        }

        if(arm.getAngle().getDegrees() < 98)
        {
            commandList.add(
                new InstantCommand(() -> arm.setArmAngle(104, 0.5), arm)
            );

            commandList.add(
                new WaitUntilCommand(() -> arm.isArmToPosition())
            );

            commandList.add(
                new InstantCommand(() -> arm.setArmAngle(98, 0.2), arm)
            );
            
            commandList.add(
                new WaitUntilCommand(() -> arm.isArmToPosition()) 
            );
        }

        else{
            commandList.add(
                new InstantCommand(() -> arm.setArmAngle(92, 0.5), arm)
            );

            commandList.add(
                new WaitUntilCommand(() -> arm.isArmToPosition())
            );

            commandList.add(
                new InstantCommand(() -> arm.setArmAngle(98, 0.2), arm)
            );

            commandList.add(
                new WaitUntilCommand(() -> arm.isArmToPosition())
            );
        }

        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(5.3, 0.2), elevator)
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.isElevatorToPosition())
        );

        commandList.add(
            new WaitCommand(2.0)
        );

        commandList.add(
            new InstantCommand(() -> elevator.setElevatorHeight(3.85, 0.2), elevator)
        );

        commandList.add(
            new WaitUntilCommand(() -> elevator.isElevatorToPosition())
        );

        //Create sequential command group from list
        group = new SequentialCommandGroup(commandList.toArray(new CommandBase[commandList.size()]));
        return group;
    }
}