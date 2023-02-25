// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardDisplay extends SubsystemBase {
  /*Subsystem References for Data and Commands*/
  private Swerve m_Swerve;

  /*Camera Streams*/
  private UsbCamera turnTableCamera;
  private UsbCamera armCamera;

  /*Shuffleboard Tab*/
  private ShuffleboardTab competitionTab;

  /*Shuffleboard Data*/

  private GenericEntry gyroAngle;
  private GenericEntry targetType;
  private GenericEntry scoreLocation;
  private GenericEntry targetDistance;
  
  private Field2d field = new Field2d();


  public DashboardDisplay(Swerve m_Swerve) {
    /*Subsystem Instantiation */
    this.m_Swerve = m_Swerve;
    competitionTab = Shuffleboard.getTab("Competition");

    turnTableCamera = CameraServer.startAutomaticCapture(0);
    armCamera = CameraServer.startAutomaticCapture(1);

    turnTableCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 10);
    armCamera.setVideoMode(PixelFormat.kMJPEG, 480, 320, 10);

    competitionTab.add("Turntable", turnTableCamera);
    competitionTab.add("Arm", armCamera);

    /*Shuffleboard Data Instantiation*/
    gyroAngle = competitionTab.add("Gyro Angle", 0).getEntry();
    competitionTab.add(field);
  }

  @Override
  public void periodic() {
    field.setRobotPose(m_Swerve.getPose());
    gyroAngle.setDouble(m_Swerve.getYaw().getDegrees());
  }
}