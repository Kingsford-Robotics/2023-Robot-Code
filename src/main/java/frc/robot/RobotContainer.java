// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.RobotConstants.Turntable;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JetsonXavier;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Power;
import frc.robot.commands.ArmElevatorPositions;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.DashboardDisplay;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    /* Subsystems */
    private final Swerve m_Swerve = new Swerve();

    private final Arm m_Arm = new Arm();
    private final Elevator m_Elevator = new Elevator(0);

    private final Turntable m_Turntable = new Turntable();
    private final Intake m_Intake = new Intake();

    private final JetsonXavier m_JetsonXavier = new JetsonXavier();
    private final Limelight m_Limelight = new Limelight();
    
    private final DashboardDisplay m_Display = new DashboardDisplay(m_Swerve);

    private final Power m_Power = new Power();

    private final PneumaticsControlModule pcm = new PneumaticsControlModule(1);

    /* Commands */
    private final ArmElevatorPositions m_ArmElevatorPositions = new ArmElevatorPositions(m_Arm, m_Elevator);
    private final DeployIntake m_DeployIntake = new DeployIntake(m_Intake);
    
    /*Pathplanner Setup*/
    private FollowPathWithEvents autoCommand = null;
    private final HashMap<String, Command> eventMap = new HashMap<>();
    
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
                        () -> OIConstants.centerOfRotation.getAsInt())
                    );
        
        m_Arm.setDefaultCommand(
            new InstantCommand(() -> m_Arm.setArmSpeed(OIConstants.armSpeed.getAsDouble() * 0.3), m_Arm)
        );
            
        m_Elevator.setDefaultCommand(
            new InstantCommand(() -> m_Elevator.setElevatorSpeed(OIConstants.elevatorSpeed.getAsDouble() * 0.3), m_Elevator)
        );

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
        OIConstants.intakeDeploy.whileTrue(m_DeployIntake);
        // Reset Gyro when button is pressed
        OIConstants.resetGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));
        OIConstants.openClaw.onTrue(null);
        OIConstants.closeClaw.onTrue(null);
        OIConstants.alignPlace.whileTrue(null);
        OIConstants.groundPickup.whileTrue(null);
        OIConstants.turntablePickup.onTrue(null);
        OIConstants.armHome.onTrue(null);
        OIConstants.increaseLevel.onTrue(null);
        OIConstants.decreaseLevel.onTrue(null);

        OIConstants.toggleAutoAlign.onTrue(null);
        OIConstants.toggleConeCube.onTrue(null);
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