// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import frc.robot.Constants.VisionConstants.CamConfig;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIOHardware;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.commands.drive.DriveClosedLoopTeleop;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final Vision m_vision = new Vision();

    private final DriveSubsystem m_drive;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.Driver.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        for (CamConfig config : CamConfig.values()) {
            m_vision.addCamera(config);
        }

        m_drive = new DriveSubsystem(
            new GyroIOSim(), 
            new ModuleIOSim(ModuleConfig.FrontLeft), 
            new ModuleIOSim(ModuleConfig.FrontRight), 
            new ModuleIOSim(ModuleConfig.RearRight),
            new ModuleIOSim(ModuleConfig.RearLeft),
            m_vision
        );

        configureBindings();
    }

    /**
    * Use this method to define your trigger->command mappings. Triggers can be created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
    private void configureBindings() {
        Pose2d targetPose = new Pose2d(m_vision.getClosestAprilTagCoordinates(m_drive.getPose().getX(), m_drive.getPose().getY())[0], m_vision.getClosestAprilTagCoordinates(m_drive.getPose().getX(), m_drive.getPose().getY())[1], Rotation2d.fromDegrees(180));
        PathConstraints constraints = new PathConstraints(null, null, null, null);
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));
        m_drive.setDefaultCommand(new DriveClosedLoopTeleop(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.Driver.kControllerDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.Driver.kControllerDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.Driver.kControllerDeadband),
            () -> m_driverController.rightBumper().getAsBoolean(),
            () -> m_driverController.leftBumper().getAsBoolean(),
            m_drive));
            // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
            // cancelling on release.

        m_driverController.b().whileTrue(new PathPlannerAuto("ODTAUTO2"));
        m_driverController.a().whileTrue(new AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0,
            0.0
        ));

    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new PathPlannerAuto("ODTAUTO2");
    }
}
