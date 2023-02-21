// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JetsonXavier extends SubsystemBase {
  public enum targetType
  {
    CONE,
    CUBE
  }
  public class targetInfo
  {
    public double[] position;
    public boolean trackingStatus;
    public targetType type;
    public double confidence;
  }
  
  //Check arraylist syntax.
  private ArrayList<targetInfo> targets;
  private double gyroAngle;
  private double[] robotPose;
  private boolean isTracking;
  
  /** Creates a new Jetson Xavier. */
  public JetsonXavier() {
    targets = new ArrayList<targetInfo>();
    gyroAngle = 0.0;
    robotPose = new double[3];
    isTracking = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
