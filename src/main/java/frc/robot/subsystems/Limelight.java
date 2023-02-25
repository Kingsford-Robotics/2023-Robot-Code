// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  
  public enum LedMode {
    kOff(0), kOn(1), kBlink(2);

    public final int value;

    LedMode(int value) {
      this.value = value;
    }
  }

  private LedMode ledMode;
  private int tx;
  private int ty;

  private double distance;

  private NetworkTableInstance inst;
  private NetworkTable limelightTable;

  public Limelight() {
    ledMode = LedMode.kOff;
    tx = 0;
    ty = 0;
    distance = 0;

    inst = NetworkTableInstance.getDefault();
    limelightTable = inst.getTable("limelight");
  }

  public void setLedMode(LedMode ledMode) {
    this.ledMode = ledMode;
  }

  public int getTx() {
    return tx;
  }

  public int getTy() {
    return ty;
  }

  public double getDistance() {
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
