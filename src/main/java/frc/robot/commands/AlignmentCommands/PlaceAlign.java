// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlignmentCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.JetsonXavier;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class PlaceAlign extends CommandBase {
  
  private Swerve m_Swerve;
  private Limelight m_Limelight;
  private JetsonXavier m_JetsonXavier;
  
  public PlaceAlign(Swerve m_Swerve, Limelight m_Limelight, JetsonXavier m_JetsonXavier) {
    this.m_Swerve = m_Swerve;
    this.m_Limelight = m_Limelight;
    this.m_JetsonXavier = m_JetsonXavier;

    addRequirements(m_Swerve, m_Limelight, m_JetsonXavier);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
