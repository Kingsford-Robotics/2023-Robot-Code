// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LogitechJoystick;

/** Add your docs here. */
public class OIConstants {
    public static final Joystick driveJoystickLeft = new Joystick(0);
    public static final Joystick driveJoystickRight = new Joystick(1);

    public static final Supplier<Double> translationSupplier = () -> driveJoystickRight.getRawAxis(Joystick.kDefaultYChannel);
    public static final Supplier<Double> strafeSupplier = () -> driveJoystickRight.getRawAxis(Joystick.kDefaultXChannel);
    public static final Supplier<Double> rotationSupplier = () -> driveJoystickLeft.getRawAxis(Joystick.kDefaultXChannel);
    public static final BooleanSupplier slowSpeed = () -> driveJoystickRight.getRawButton(LogitechJoystick.thumbButton);
    public static final BooleanSupplier robotCentric = () -> driveJoystickLeft.getRawButton(LogitechJoystick.trigger);

    public static final double highTranslationSpeed = 1.0;
    public static final double lowTranslationSpeed = 0.25;

    public static final double highRotationSpeed = 1.0;
    public static final double lowRotationSpeed = 0.25;

    public static final double translationDeadBand = 0.1;
    public static final double turnDeadBand = 0.1;

    public static final JoystickButton intakeDeploy = new JoystickButton(driveJoystickRight, LogitechJoystick.trigger);    //Intake deploy while held. Retract when released.
    public static final JoystickButton resetGyro = new JoystickButton(driveJoystickRight, LogitechJoystick.button3);    //Resets the gyro to 0 degrees.

    //Sets the center of the rotation to the selected wheel while held. Returns to center of robot when released.
    public static final JoystickButton centerOfRotationFrontLeft = new JoystickButton(driveJoystickLeft, LogitechJoystick.button5);
    public static final JoystickButton centerOfRotationFrontRight = new JoystickButton(driveJoystickLeft, LogitechJoystick.button6); 
    public static final JoystickButton centerOfRotationBackLeft = new JoystickButton(driveJoystickLeft, LogitechJoystick.button3);
    public static final JoystickButton centerOfRotationBackRight = new JoystickButton(driveJoystickLeft, LogitechJoystick.button4);

    public static final XboxController controller = new XboxController(2);

    public static final Supplier<Double> armSpeed = () ->controller.getRawAxis(XboxController.Axis.kLeftY.value);
}