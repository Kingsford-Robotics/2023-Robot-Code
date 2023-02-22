// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmElevator;

public class TeleopArmElevator extends CommandBase {
  /** Creates a new TeleopArmElevator. */
  ArmElevator armElevator;
  public TeleopArmElevator(ArmElevator armElevator) {
    this.armElevator = armElevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Set arm and elevator speed to joystick values from OIConstants suppliers
    armElevator.setArmPercent(OIConstants.armSpeed.get());
    armElevator.setElevatorPercent(-OIConstants.elevatorSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
