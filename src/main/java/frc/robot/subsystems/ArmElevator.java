// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElevator extends SubsystemBase {
  //Subsystem that coordinates the use of the arm and elevator subsystems.
  //Ensures that the arm and elevator are in valid positions at all times.

  private Arm arm;
  private Elevator elevator;

  public ArmElevator() {
    arm = new Arm();
    elevator = new Elevator();
  }

  private double getArmMaxPosition(double elevatorHeight, boolean armExtended){
    //Returns the maximum valid arm position.
    return 0.0;
  }

  private double getArmMinPosition(double elevatorHeight, boolean armExtended){
    //Returns the minimum valid arm position.
    return 0.0;
  }

  public boolean setArmPosition(double position){
    double maxValidPosition = 0.0;
    double minValidPosition = 0.0;

    //Check if the arm is in a valid position.
    if (position > maxValidPosition || position < minValidPosition){
      return false;
    }

    arm.setArmPosition(position);
    return true;
  }

  public boolean setElevatorPosition(double position){
    elevator.setElevatorPosition(position);
    return true;
  }

  public void setArmSpeed(double speed){
    arm.setArmSpeedPercent(speed);
  }

  public void setElevatorSpeed(double speed){
    elevator.setElevatorSpeed(speed);
  }

  //Used to directly access arm methods.
  public Arm getArm(){
    return arm;
  }

  //Used to directly access elevator methods.
  public Elevator getElevator(){
    return elevator;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
