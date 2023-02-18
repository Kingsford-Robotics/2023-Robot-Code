// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardDisplay extends SubsystemBase {
  /*Subsystems for Data and Commands*/
  private Swerve m_Swerve;

  private ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
  //private ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
  //private ShuffleboardTab testTab = Shuffleboard.getTab("Test");

  private GenericEntry gyroAngle = competitionTab.add("Gyro", 0).getEntry();

  private Field2d field = new Field2d();

  public DashboardDisplay(Swerve m_Swerve) {
    /*Subsystem Instantiation */
    this.m_Swerve = m_Swerve;
    competitionTab.add(field);
  }

  @Override
  public void periodic() {
    field.setRobotPose(m_Swerve.getPose());
    gyroAngle.setDouble(m_Swerve.getYaw().getDegrees());
  }
}
