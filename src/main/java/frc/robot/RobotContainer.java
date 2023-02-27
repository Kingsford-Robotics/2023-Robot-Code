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
import frc.robot.subsystems.Turntable;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JetsonXavier;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Ramp;
import frc.robot.commands.ArmPickupAlign;
import frc.robot.commands.PlaceAlign;
import frc.robot.commands.SetElevatorToHeight;
import frc.robot.commands.StopArmElevator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.DashboardDisplay;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    /* Subsystems */
    private final Swerve m_Swerve = new Swerve();

    private final Arm m_Arm = new Arm();
    private final Elevator m_Elevator = new Elevator(0);

    private final Turntable m_Turntable = new Turntable();
    private final Ramp m_Ramp = new Ramp();

    private final JetsonXavier m_JetsonXavier = new JetsonXavier();
    private final Limelight m_Limelight = new Limelight();
    
    private final DashboardDisplay m_Display = new DashboardDisplay(this, m_Swerve);

    private final PneumaticsControlModule pcm = new PneumaticsControlModule(1);

    /* Commands */
    private final StopArmElevator m_StopArmElevator = new StopArmElevator(m_Arm, m_Elevator);
    private final PlaceAlign m_PlacewAlign = new PlaceAlign(m_Swerve, m_Limelight, m_JetsonXavier);
    private final ArmPickupAlign m_ArmPickupAlign = new ArmPickupAlign(m_Swerve, m_JetsonXavier);
    private final SetElevatorToHeight elevatorToHeight = new SetElevatorToHeight(m_Elevator, 3.0);

    /*Pathplanner Setup*/
    private FollowPathWithEvents autoCommand = null;
    private final HashMap<String, Command> eventMap = new HashMap<>();

    /*Control State Variables*/
    private int level = 2;    //Levels 0 - 2 represent FLOOR, MIDDLE, and TOP
    private boolean isCone = true;
    private boolean autoAlign = true;
    
    public int getLevel() { return level; }

    public void setLevel(int level) { this.level = level; }

    public boolean getIsCone() { return isCone; }

    public void setIsCone(boolean isCone) { this.isCone = isCone; }

    public boolean getAutoAlign() { return autoAlign; }

    public void setAutoAlign(boolean autoAlign) { this.autoAlign = autoAlign; }

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
            new InstantCommand(() -> m_Arm.setArmSpeed(OIConstants.armSpeed.getAsDouble() * 0.2), m_Arm)
        );
            
        m_Elevator.setDefaultCommand(
            new InstantCommand(() -> m_Elevator.setElevatorSpeed(-OIConstants.elevatorSpeed.getAsDouble() * 0.3), m_Elevator)
        );

        m_Turntable.setDefaultCommand(
            new InstantCommand(() -> m_Turntable.setTurntableSpeed(OIConstants.turntableSpeed.getAsDouble() * 1.0), m_Turntable)
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
        OIConstants.resetGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));

        OIConstants.openClaw.onTrue(new InstantCommand(() -> m_Arm.open()));
        OIConstants.closeClaw.onTrue(new InstantCommand(() -> m_Arm.close()));

        OIConstants.increaseLevel.onTrue(new InstantCommand(() -> level = Math.min(level + 1, 2)));
        OIConstants.decreaseLevel.onTrue(new InstantCommand(() -> level = Math.max(level - 1, 0)));

        OIConstants.toggleConeCube.onTrue(new InstantCommand(() -> isCone = !isCone));

        OIConstants.alignPlace.whileTrue(elevatorToHeight);
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

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }
}