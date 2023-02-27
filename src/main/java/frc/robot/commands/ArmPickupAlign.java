// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.JetsonXavier;
import frc.robot.subsystems.Swerve;

public class ArmPickupAlign extends CommandBase {
  
  private Swerve m_Swerve;
  private JetsonXavier m_JetsonXavier;

  public ArmPickupAlign(Swerve m_Swerve, JetsonXavier m_JetsonXavier) {
    this.m_Swerve = m_Swerve;
    this.m_JetsonXavier = m_JetsonXavier;

    addRequirements(m_Swerve, m_JetsonXavier);
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
