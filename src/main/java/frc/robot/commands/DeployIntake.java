// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.chrono.ThaiBuddhistChronology;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AlignmentCommands.IntakeAlign;
import frc.robot.subsystems.Intake;

public class DeployIntake extends CommandBase {
  
  private Intake m_Intake = new Intake();
  private IntakeAlign m_IntakeAlign;

  public DeployIntake(Intake m_Intake, IntakeAlign m_IntakeAlign) {
    this.m_Intake = m_Intake;
    this.m_IntakeAlign = m_IntakeAlign;

    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.deployIntake();
    m_Intake.intakeIn();
    m_IntakeAlign.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    //TODO: Add left-right alignment code using Jetson Xavier data.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.retractIntake();
    m_Intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
