// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {
  
  private VictorSPX intakeMotor;
  private DoubleSolenoid intakeDeploySolenoid;

  /*Shuffleboard Setup*/
  private ShuffleboardTab intakeTab;
  
  private GenericEntry intakeDeployed;
  private GenericEntry intakeSpeed;


  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new VictorSPX(RobotConstants.Intake.intakeMotorID);
    intakeDeploySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotConstants.Intake.intakeSolenoidFWD, RobotConstants.Intake.intakeSolenoidREV);

    /*Shuffleboard Setup*/
    intakeTab = Shuffleboard.getTab("Intake");
    intakeDeployed = intakeTab.add("Intake Deployed", false).getEntry();
    intakeSpeed = intakeTab.add("Intake Speed", 0).getEntry();
  }

  public void deployIntake() {
    intakeDeploySolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake() {
    intakeDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleIntake(){
    intakeDeploySolenoid.toggle();
  }

  public void intakeIn() {
    intakeMotor.set(ControlMode.PercentOutput, RobotConstants.Intake.intakeSpeed);
  }

  public void intakeOut() {
    intakeMotor.set(ControlMode.PercentOutput, RobotConstants.Intake.outtakeSpeed);
  }

  public boolean isIntakeDeployed() {
    return intakeDeploySolenoid.get() == DoubleSolenoid.Value.kForward;
  }

  public void stopIntake() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    /*Shuffleboard Output*/
    intakeDeployed.setBoolean(isIntakeDeployed());
    intakeSpeed.setDouble(intakeMotor.getMotorOutputPercent());
  }
}
