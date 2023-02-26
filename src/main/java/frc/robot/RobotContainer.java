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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.RobotConstants.Turntable;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.JetsonXavier;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.ArmElevatorPositions;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.StopArmElevator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.AlignmentCommands.ArmPickupAlign;
import frc.robot.commands.AlignmentCommands.IntakeAlign;
import frc.robot.commands.AlignmentCommands.PlaceAlign;
import frc.robot.subsystems.DashboardDisplay;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    /* Subsystems */
    private final Swerve m_Swerve = new Swerve();

    private final Arm m_Arm = new Arm();
    private final Elevator m_Elevator = new Elevator(0);

    private final Turntable m_Turntable = new Turntable();
    //private final Intake m_Intake = new Intake();

    private final JetsonXavier m_JetsonXavier = new JetsonXavier();
    private final Limelight m_Limelight = new Limelight();
    
    private final DashboardDisplay m_Display = new DashboardDisplay(m_Swerve);

    private final PneumaticsControlModule pcm = new PneumaticsControlModule(1);

    /* Commands */
    private final ArmElevatorPositions m_ArmElevatorPositions = new ArmElevatorPositions(m_Arm, m_Elevator);
    //private final ReverseIntake m_ReverseIntake = new ReverseIntake(m_Intake);
    private final StopArmElevator m_StopArmElevator = new StopArmElevator(m_Arm, m_Elevator);

    private final PlaceAlign m_PlacewAlign = new PlaceAlign(m_Swerve, m_Limelight, m_JetsonXavier);
    private final ArmPickupAlign m_ArmPickupAlign = new ArmPickupAlign(m_Swerve, m_Limelight, m_JetsonXavier);
    private final IntakeAlign m_IntakeAlign = new IntakeAlign(m_Swerve, m_Limelight, m_JetsonXavier);

    //private final DeployIntake m_DeployIntake = new DeployIntake(m_Intake, m_IntakeAlign);

    /*Pathplanner Setup*/
    private FollowPathWithEvents autoCommand = null;
    private final HashMap<String, Command> eventMap = new HashMap<>();

    private int level = 2;    //Levels 0 - 2 represent FLOOR, MIDDLE, and TOP
    private boolean isCone = true;
    private boolean autoAlign = true;
    
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
        //OIConstants.intakeDeploy.whileTrue(m_DeployIntake);
        OIConstants.resetGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));

        OIConstants.openClaw.onTrue(new InstantCommand(() -> m_Arm.open()));
        OIConstants.closeClaw.onTrue(new InstantCommand(() -> m_Arm.close()));

        OIConstants.toggleArmExtension.onTrue(new InstantCommand(() -> m_Arm.toggleExtension()));

        OIConstants.alignPlace.whileTrue(
            new ParallelCommandGroup(
                m_PlacewAlign,    
                new SequentialCommandGroup(
                    new PrintCommand("Print Something"),
                    new PrintCommand("Print Something Else"))
                )  
        );
        
        OIConstants.alignPlace.onFalse(m_StopArmElevator);

        /*OIConstants.groundPickup.whileTrue(
            new ParallelCommandGroup(
                m_ArmPickupAlign,
                m_ArmElevatorPositions.getArmElevatorCommand(ArmElevatorPositions.Positions.GROUND_PICKUP)
            )
        );*/

        OIConstants.groundPickup.onFalse(m_StopArmElevator);

        //OIConstants.turntablePickup.onTrue(m_ArmElevatorPositions.getArmElevatorCommand(ArmElevatorPositions.Positions.TURNTABLE_PICKUP));
        OIConstants.turntablePickup.onFalse(m_StopArmElevator);
        
        //OIConstants.armHome.onTrue(m_ArmElevatorPositions.getArmElevatorCommand(ArmElevatorPositions.Positions.HOME));
        OIConstants.armHome.onFalse(m_StopArmElevator);

        //OIConstants.reverseIntake.onTrue(m_ReverseIntake);

        OIConstants.increaseLevel.onTrue(new InstantCommand(() -> level = Math.min(level++, 2)));
        OIConstants.decreaseLevel.onTrue(new InstantCommand(() -> level = Math.max(level--, 0)));

        OIConstants.toggleAutoAlign.onTrue(new InstantCommand(() -> autoAlign = !autoAlign));
        OIConstants.toggleConeCube.onTrue(new InstantCommand(() -> isCone = !isCone));
    }

    private ArmElevatorPositions.Positions getPosition(int level, boolean isCone)
    {
        if(isCone)
        {
            switch(level)
            {
                case 0:
                    return ArmElevatorPositions.Positions.CONE_FLOOR;
                case 1:
                    return ArmElevatorPositions.Positions.CONE_MIDDLE;
                case 2:
                    return ArmElevatorPositions.Positions.CONE_TOP;
                default:
                    return ArmElevatorPositions.Positions.HOME;
            }
        }

        else{
            switch(level)
            {
                case 0:
                    return ArmElevatorPositions.Positions.CUBE_FLOOR;
                case 1:
                    return ArmElevatorPositions.Positions.CUBE_MIDDLE;
                case 2:
                    return ArmElevatorPositions.Positions.CUBE_TOP;
                default:
                    return ArmElevatorPositions.Positions.HOME;
            }
        }
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