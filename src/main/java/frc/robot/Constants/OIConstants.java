// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LogitechJoystick;

/** Add your docs here. */
public class OIConstants {

    /*Drive Joysticks Setup*/
    public static final Joystick driveJoystickLeft = new Joystick(0);
    public static final Joystick driveJoystickRight = new Joystick(1);

    //Drive Joysticks Suppliers
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

    //Drive Joysticks Buttons
    public static final JoystickButton intakeDeploy = new JoystickButton(driveJoystickRight, LogitechJoystick.trigger);    //Intake deploy while held. Retract when released.
    public static final JoystickButton resetGyro = new JoystickButton(driveJoystickRight, LogitechJoystick.button3);    //Resets the gyro to 0 degrees.

    //Sets the center of the rotation to the selected wheel while held. Returns to center of robot when released.
    private static final JoystickButton centerOfRotationFrontLeft = new JoystickButton(driveJoystickLeft, LogitechJoystick.button5);
    private static final JoystickButton centerOfRotationFrontRight = new JoystickButton(driveJoystickLeft, LogitechJoystick.button6); 
    private static final JoystickButton centerOfRotationBackLeft = new JoystickButton(driveJoystickLeft, LogitechJoystick.button3);
    private static final JoystickButton centerOfRotationBackRight = new JoystickButton(driveJoystickLeft, LogitechJoystick.button4);

    public static final IntSupplier centerOfRotation = () -> {
        if (OIConstants.centerOfRotationFrontLeft.getAsBoolean()) {
            return 0;
        } else if (OIConstants.centerOfRotationFrontRight.getAsBoolean()) {
            return 1;
        } else if (OIConstants.centerOfRotationBackLeft.getAsBoolean()) {
            return 2;
        } else if (OIConstants.centerOfRotationBackRight.getAsBoolean()) {
            return 3;
        } else {
            return -1;
        }
    };

    /*Operator Joystick Setup*/
    public static final XboxController handheldController = new XboxController(2);
    public static final XboxController mountedController = new XboxController(3);

    //Handheld Controller
    public static final JoystickButton openClaw = new JoystickButton(handheldController, XboxController.Button.kLeftBumper.value);
    public static final JoystickButton closeClaw = new JoystickButton(handheldController, XboxController.Button.kRightBumper.value);

    public static final JoystickButton alignPlace = new JoystickButton(handheldController, XboxController.Button.kY.value);
    public static final JoystickButton groundPickup = new JoystickButton(handheldController, XboxController.Button.kB.value);
    public static final JoystickButton turntablePickup = new JoystickButton(handheldController, XboxController.Button.kX.value);
    public static final JoystickButton armHome = new JoystickButton(handheldController, XboxController.Button.kA.value);

    public static final DoubleSupplier elevatorSpeed = () -> handheldController.getLeftY();
    public static final DoubleSupplier armSpeed = () -> handheldController.getRightY();

    public static final int turntableLeftPOV = 270;
    public static final int turntableRightPOV = 90;

    //Mounted Controller
    public static final JoystickButton increaseLevel = new JoystickButton(mountedController, XboxController.Button.kY.value);
    public static final JoystickButton decreaseLevel = new JoystickButton(mountedController, XboxController.Button.kA.value);
    
    public static final JoystickButton toggleAutoAlign = new JoystickButton(mountedController, XboxController.Button.kB.value);
    public static final JoystickButton toggleConeCube = new JoystickButton(mountedController, XboxController.Button.kX.value);
}