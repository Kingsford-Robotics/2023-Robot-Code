// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.function.IntSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmElevator;
import frc.robot.commands.ArmElevatorPositions;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.DashboardDisplay;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final IntSupplier centerOfRotation = () -> {
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
    /* Subsystems */
    private final Swerve m_Swerve = new Swerve();
    private final ArmElevator m_ArmElevator = new ArmElevator();
    private final DashboardDisplay m_Display = new DashboardDisplay(m_Swerve);
    private final Pneumatics m_Pneumatics = new Pneumatics();
    
    FollowPathWithEvents autoCommand = null;

    /* Commands */
    private final ArmElevatorPositions m_ArmElevatorPositions = new ArmElevatorPositions(m_ArmElevator);
    
    HashMap<String, Command> eventMap = new HashMap<>();
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        m_Swerve,
                        () -> -OIConstants.translationSupplier.get(),
                        () -> -OIConstants.strafeSupplier.get(),
                        () -> -OIConstants.rotationSupplier.get(),
                        () -> OIConstants.robotCentric.getAsBoolean(),
                        () -> OIConstants.slowSpeed.getAsBoolean(),
                        () -> centerOfRotation.getAsInt()));
        
        // Configure the button bindings
        configureButtonBindings();
        configureAutoCommands();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Reset Gyro when button is pressed
        OIConstants.resetGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));
    }

    private void configureAutoCommands()
    {
        eventMap.put("event1", new PrintCommand("Event 1"));
        eventMap.put("event2", new PrintCommand("Event 2"));

        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(3, 3));
    
        autoCommand = new FollowPathWithEvents(
            m_Swerve.followTrajectoryCommand(pathGroup.get(0), true),
            pathGroup.get(0).getMarkers(), eventMap
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }
}