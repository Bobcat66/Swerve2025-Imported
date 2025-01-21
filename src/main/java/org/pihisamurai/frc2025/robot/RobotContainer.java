// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pihisamurai.frc2025.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants.ppConfig;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.pihisamurai.frc2025.robot.Constants.Akit;
import org.pihisamurai.frc2025.robot.Constants.OIConstants;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import org.pihisamurai.frc2025.robot.Constants.VisionConstants.CamConfig;
import org.pihisamurai.frc2025.robot.commands.ExampleCommand;
import org.pihisamurai.frc2025.robot.commands.drive.DriveClosedLoopTeleop;
import org.pihisamurai.frc2025.robot.subsystems.ExampleSubsystem;
import org.pihisamurai.frc2025.robot.subsystems.Vision;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;
import org.pihisamurai.frc2025.robot.subsystems.drive.GyroIOHardware;
import org.pihisamurai.frc2025.robot.subsystems.drive.GyroIOSim;
import org.pihisamurai.frc2025.robot.subsystems.drive.ModuleIOHardware;
import org.pihisamurai.frc2025.robot.subsystems.drive.ModuleIOSim;
import org.pihisamurai.frc2025.robot.utils.Localization;
import org.pihisamurai.lib.debug.DebugLib.FieldMonitor;
import org.pihisamurai.lib.debug.DebugLib.ObjectMonitor;
import org.pihisamurai.lib.debug.DebugLib.PPDebugging;
import org.pihisamurai.lib.debug.DebugLib.ReflectionDebugger;
import org.pihisamurai.lib.debug.DebugLib.ReflectionDebugger.ClassMonitor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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

        //Pose2d targetPose = new Pose2d(Localization.getClosestAprilTagCoordinates(m_drive.getPose().getX(), m_drive.getPose().getY())[0], Localization.getClosestAprilTagCoordinates(m_drive.getPose().getX(), m_drive.getPose().getY())[1], Rotation2d.fromDegrees(180));
        Pose2d targetPose = Localization.getClosestReefFace(m_drive.getPose()).AprilTag;
        PathConstraints constraints = new PathConstraints(MetersPerSecond.of(5), MetersPerSecondPerSecond.of(5), RadiansPerSecond.of(5), RadiansPerSecondPerSecond.of(5));
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

        m_driverController.b().whileTrue(new PathPlannerAuto("AUTO1"));
        m_driverController.a().whileTrue(AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0
        ));

    }

    /**
     * Returns a modified PathPlannerAuto for debugging
     * @author Jesse Kane
     */
    /* 
    private Command getDebugAutoCommand() { 
        PathPlannerAuto auto = new PathPlannerAuto("AUTO1");
        SequentialCommandGroup autoSCG = (SequentialCommandGroup)PPDebugging.getAutoCommand(auto);
        FollowPathCommand autoFPC = (FollowPathCommand)PPDebugging.getNthCommandFromSCG(autoSCG, 0).get();
        PathPlannerTrajectory trajectory = PPDebugging.getTrajectoryFromFPC(autoFPC);
        ClassMonitor FPCMonitor = ReflectionDebugger.getInstance().getClassMonitor(FollowPathCommand.class);

        
        return new Command() {

            FollowPathCommand fpc = autoFPC;
            
            ObjectMonitor<Timer> timerMntr = FPCMonitor.<Timer>getObjectMonitorFromField("timer", autoFPC);
            ObjectMonitor<PathPlannerPath> originalPathMntr = FPCMonitor.<PathPlannerPath>getObjectMonitorFromField("originalPath",autoFPC);
            ObjectMonitor<Supplier<Pose2d>> poseSupplierMntr = FPCMonitor.<Supplier<Pose2d>>getObjectMonitorFromField("poseSupplier",autoFPC);
            ObjectMonitor<Supplier<ChassisSpeeds>> speedsSupplierMntr = FPCMonitor.<Supplier<ChassisSpeeds>>getObjectMonitorFromField("speedsSupplier",autoFPC);
            ObjectMonitor<BiConsumer<ChassisSpeeds, DriveFeedforwards>> outputMntr = FPCMonitor.<BiConsumer<ChassisSpeeds,DriveFeedforwards>>getObjectMonitorFromField("output",autoFPC);
            ObjectMonitor<PathFollowingController> controllerMntr = FPCMonitor.<PathFollowingController>getObjectMonitorFromField("controller", autoFPC);
            ObjectMonitor<RobotConfig> robotConfigMntr = FPCMonitor.<RobotConfig>getObjectMonitorFromField("robotConfig",autoFPC);
            ObjectMonitor<BooleanSupplier> shouldFlipPathMntr = FPCMonitor.<BooleanSupplier>getObjectMonitorFromField("shouldFlipPath",autoFPC);
            ObjectMonitor<EventScheduler> eventSchedulerMntr = FPCMonitor.<EventScheduler>getObjectMonitorFromField("eventScheduler",autoFPC);
            ObjectMonitor<PathPlannerPath> pathMntr = FPCMonitor.<PathPlannerPath>getObjectMonitorFromField("path",autoFPC);
            ObjectMonitor<PathPlannerTrajectory> trajectoryMntr = FPCMonitor.<PathPlannerTrajectory>getObjectMonitorFromField("trajectory", autoFPC);
            
            Function<Double,PathPlannerTrajectoryState> sampleTrajectory = (Double time) -> {

                if (time <= trajectoryMntr.get().getInitialState().timeSeconds) return trajectoryMntr.get().getInitialState();
                if (time >= trajectoryMntr.get().getTotalTimeSeconds()) return trajectoryMntr.get().getEndState();

                System.out.println("Calculating low and high");

                int low = 1;
                int high = trajectoryMntr.<List<PathPlannerTrajectoryState>>getField("states").get().size() - 1;
                while (low != high) {
                    System.out.println("Running sampling loop");
                    int mid = (low + high) / 2;
                    System.out.println("Low: " + low);
                    System.out.println("Mid: " + mid);
                    System.out.println("High: " + high);
                    if (trajectoryMntr.get().getState(mid).timeSeconds < time) {
                        low = mid + 1;
                    } else {
                        high = mid;
                    }
                }

                System.out.println("Getting Samples");
                var sample = trajectoryMntr.get().getState(low);
                var prevSample = trajectoryMntr.get().getState(low - 1);
                System.out.println("States:");
                List<PathPlannerTrajectoryState> states = trajectoryMntr.get().getStates();
                //trajectoryMntr.getStaticClosure("forwardAccelPass{List,RobotConfig}").invoke(states,ppConfig);
                for (var state : states){
                    System.out.println(state.fieldSpeeds);
                }
                if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-3) {
                    return sample;
                }
                System.out.println("time - prevSample.timeSeconds: " + (time - prevSample.timeSeconds));
                System.out.println("sample.timeSeconds - prevSample.timeSeconds: " + (sample.timeSeconds - prevSample.timeSeconds));
                System.out.println("Interpolating prevSample");
                return prevSample.interpolate(sample, (time - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
            };

            {
                System.out.println("Executing command initialization block");
                ObjectMonitor<CommandScheduler> csMntr = ObjectMonitor.of(CommandScheduler.getInstance());
                csMntr.<Set<Command>>getField("m_scheduledCommands").get().remove(autoFPC);
                csMntr.<Set<Command>>getField("m_endingCommands").get().remove(autoFPC);
                csMntr.<Map<Subsystem,Command>>getField("m_requirements").get().keySet().removeAll(autoFPC.getRequirements());
                addRequirements(m_drive);
            }

            @Override
            public void initialize() {
                autoFPC.initialize();
            }

            @Override
            public void execute() {
                //fpc.execute();
                
                System.out.println((double) Integer.MAX_VALUE);
                double currentTime = timerMntr.get().get();
                System.out.println("Sampling trajectory at " + currentTime);
                var targetState = sampleTrajectory.apply(currentTime);
                System.out.println("Sampled trajectory");
                if (!controllerMntr.get().isHolonomic() && pathMntr.get().isReversed()) {
                    targetState = targetState.reverse();
                }

                Pose2d currentPose = poseSupplierMntr.get().get();
                ChassisSpeeds currentSpeeds = speedsSupplierMntr.get().get();

                ChassisSpeeds targetSpeeds = controllerMntr.get().calculateRobotRelativeSpeeds(currentPose, targetState);

                System.out.println("targetSpeeds: " + targetSpeeds);

                double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

                PPLibTelemetry.setCurrentPose(currentPose);
                PathPlannerLogging.logCurrentPose(currentPose);

                PPLibTelemetry.setTargetPose(targetState.pose);
                PathPlannerLogging.logTargetPose(targetState.pose);

                PPLibTelemetry.setVelocities(
                    currentVel,
                    targetState.linearVelocity,
                    currentSpeeds.omegaRadiansPerSecond,
                    targetSpeeds.omegaRadiansPerSecond);

                outputMntr.get().accept(targetSpeeds, targetState.feedforwards);

                eventSchedulerMntr.get().execute(currentTime);
                // 
            }

            @Override
            public boolean isFinished(){
                return fpc.isFinished();
            }
            @Override
            public void end(boolean interrupted){
                fpc.end(interrupted);
            }
        };
    }
    */

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        //return getDebugAutoCommand();
        return AutoBuilder.buildAuto("AUTO1");
        //return getDebugFollowPathCommand();
    }
}
