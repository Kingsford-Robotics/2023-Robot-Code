// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Arm extends SubsystemBase {
    /** Creates a new Elevator. */
    private TalonFX armMotor;
    private CANCoder angleEncoder;

    private DoubleSolenoid armExtension;
    private DoubleSolenoid armGrab;

    private ArmFeedforward feedforward;
    private ProfiledPIDController controller;

    public Arm() {
        armMotor = new TalonFX(RobotConstants.armMotorID);
        armMotor.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
       
        angleEncoder = new CANCoder(RobotConstants.ArmEncoderID);
        
        angleEncoder.configFactoryDefault();
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        angleEncoder.configAllSettings(encoderConfig);

        armExtension = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.armExtensionSolenoidFWD, 
            RobotConstants.armExtensionSolenoidREV
        );

        armGrab = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            RobotConstants.armGrabSolenoidFWD, 
            RobotConstants.armGrabSolenoidREV
        );

        armExtension.set(kReverse);
        armGrab.set(kForward);   
  
        feedforward = new ArmFeedforward(0.1, 0.1, 0.1);  // kS, kV, kA constants for feedforward controller 

        controller = new ProfiledPIDController(
            0.1, 
            0.0, 
            0.0,  
            new TrapezoidProfile.Constraints(
                0,
                0)
        );
    }

    double getAngle(){
        if (angleEncoder.getAbsolutePosition() - RobotConstants.ArmEncoderOffset < 0){
            return 360 + angleEncoder.getAbsolutePosition() - RobotConstants.ArmEncoderOffset;
        }
        else
        {
            return angleEncoder.getAbsolutePosition() - RobotConstants.ArmEncoderOffset;
        }
    }

    void toggleGrab(){
        armGrab.toggle();
    }

    void toggleExtension(){
        armExtension.toggle();
    }

    void setArmSpeedPercent(double speed){
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    void setArmSpeedRadiansSec(double speed)
    {
        armMotor.set(ControlMode.Velocity, speed);
    }
    
    void setArmPosition(double position){
        controller.setGoal(position);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }
}