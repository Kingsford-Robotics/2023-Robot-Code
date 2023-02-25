// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Power extends SubsystemBase {
  private final PowerDistribution pdp;
  
  /*Shuffleboard*/
  private ShuffleboardTab powerTab;

  private GenericEntry totalCurrent;
  private GenericEntry totalPower;
  private GenericEntry totalEnergy;

  private GenericEntry switchableChannelState;

  public Power() {
    powerTab = Shuffleboard.getTab("Power");
    
    totalCurrent = powerTab.add("Total Current", 0).getEntry();
    totalPower = powerTab.add("Total Power", 0).getEntry();
    totalEnergy = powerTab.add("Total Energy", 0).getEntry();

    switchableChannelState = powerTab.add("Switchable Channel State", false).getEntry();

    pdp = new PowerDistribution();
    pdp.resetTotalEnergy();
    pdp.setSwitchableChannel(false);
  }

  public void setSwitchableChannel(boolean state) {
    pdp.setSwitchableChannel(state);
  }

  public boolean getSwitchableChannel() {
    return pdp.getSwitchableChannel();
  }

  public double getTotalCurrent() {
    return pdp.getTotalCurrent();
  }

  public double getTotalPower() {
    return pdp.getTotalPower();
  }

  public double getTotalEnergy() {
    return pdp.getTotalEnergy();
  }

  @Override
  public void periodic() {
    totalCurrent.setDouble(pdp.getTotalCurrent());
    totalPower.setDouble(pdp.getTotalPower());
    totalEnergy.setDouble(pdp.getTotalEnergy());

    switchableChannelState.setBoolean(pdp.getSwitchableChannel());
  }
}