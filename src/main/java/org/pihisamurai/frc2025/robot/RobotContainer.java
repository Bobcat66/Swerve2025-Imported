// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pihisamurai.frc2025.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.doubleClutchRotationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.doubleClutchTranslationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.singleClutchRotationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.singleClutchTranslationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants.ppConfig;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.logging.Logger;

import org.pihisamurai.frc2025.robot.Constants.Akit;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants;
import org.pihisamurai.frc2025.robot.Constants.OIConstants;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.TeleopDriveMode;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PointOfInterest;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PoseOfInterest;
import org.pihisamurai.frc2025.robot.Constants.VisionConstants.CamConfig;
import org.pihisamurai.frc2025.robot.commands.ExampleCommand;
import org.pihisamurai.frc2025.robot.commands.drive.TeleopDriveCommand;
//import org.pihisamurai.frc2025.robot.commands.drive.DriveClosedLoopTeleop;
import org.pihisamurai.frc2025.robot.subsystems.ExampleSubsystem;
import org.pihisamurai.frc2025.robot.subsystems.Vision;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;
import org.pihisamurai.frc2025.robot.subsystems.drive.GyroIOHardware;
import org.pihisamurai.frc2025.robot.subsystems.drive.GyroIOSim;
import org.pihisamurai.frc2025.robot.subsystems.drive.ModuleIOHardware;
import org.pihisamurai.frc2025.robot.subsystems.drive.ModuleIOSim;
import org.pihisamurai.frc2025.robot.utils.Localization;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.Driver.kDriverControllerPort);

    private final TeleopDriveCommand teleopDrive;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    @SuppressWarnings("unused")
    public RobotContainer() {
        // Configure the trigger bindings
        if (Akit.currentMode == 0){
            for (CamConfig config : CamConfig.values()) {
                m_vision.addCamera(config);
            }
        }

        if (Akit.currentMode == 0) {
            m_drive = new DriveSubsystem(
                new GyroIOHardware(), 
                new ModuleIOHardware(ModuleConfig.FrontLeft), 
                new ModuleIOHardware(ModuleConfig.FrontRight), 
                new ModuleIOHardware(ModuleConfig.RearRight),
                new ModuleIOHardware(ModuleConfig.RearLeft),
                m_vision
            );
        } else if (Akit.currentMode == 1) {
            m_drive = new DriveSubsystem(
                new GyroIOSim(), 
                new ModuleIOSim(ModuleConfig.FrontLeft), 
                new ModuleIOSim(ModuleConfig.FrontRight), 
                new ModuleIOSim(ModuleConfig.RearRight),
                new ModuleIOSim(ModuleConfig.RearLeft),
                m_vision
            );
        }

        teleopDrive = m_drive.CommandBuilder.driveTeleop(
            () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.Driver.kControllerDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.Driver.kControllerDeadband),
            () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OIConstants.Driver.kControllerDeadband),
            TeleopDriveMode.kClosedLoopFieldOriented,
            1.0,
            1.0,
            null
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
        //System.out.println("CONFBINDINGS");

        //Pose2d targetPose = new Pose2d(Localization.getClosestAprilTagCoordinates(m_drive.getPose().getX(), m_drive.getPose().getY())[0], Localization.getClosestAprilTagCoordinates(m_drive.getPose().getX(), m_drive.getPose().getY())[1], Rotation2d.fromDegrees(180));
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        m_drive.setDefaultCommand(teleopDrive);

        m_driverController.rightBumper().whileTrue(teleopDrive.applyDoubleClutch());

        m_driverController.leftBumper().whileTrue(teleopDrive.applySingleClutch());
        
        m_driverController.leftBumper()
            .onTrue(Commands.runOnce(() -> teleopDrive.applyClutchFactors(singleClutchTranslationFactor, singleClutchRotationFactor)))
            .onFalse(Commands.runOnce(() -> teleopDrive.applyClutchFactors(1.0, 1.0)));

        m_driverController.leftTrigger(OIConstants.Driver.kControllerTriggerThreshold).whileTrue(m_drive.CommandBuilder.directDriveToNearestBranch(true, new Transform2d(0.4572, 0, Rotation2d.fromDegrees(0))));

        m_driverController.rightTrigger(OIConstants.Driver.kControllerTriggerThreshold).whileTrue(m_drive.CommandBuilder.directDriveToNearestBranch(false, new Transform2d(0.4572, 0, Rotation2d.fromDegrees(0))));

        //Drives with the heading locked to point towards the center of the alliance reef
        m_driverController.a().whileTrue(teleopDrive.applyReefHeadingLock());
        
        //Drives with heading locked to align with processor-side coral station
        m_driverController.b().whileTrue(teleopDrive.applyProcessorCoralStationHeadingLock());

        //Drives with heading locked to align with opposite-side coral station
        m_driverController.x().whileTrue(teleopDrive.applyOppositeCoralStationHeadingLock());
        
        //Drives with heading locked to align straight forward
        m_driverController.y().whileTrue(teleopDrive.applyForwardHeadingLock());
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return getDebugAutoCommand();
        return m_drive.CommandBuilder.directDriveToNearestBranch(true, new Transform2d(0.4572, 0, Rotation2d.fromDegrees(0)));
        //return getDebugFollowPathCommand();
    }
}
