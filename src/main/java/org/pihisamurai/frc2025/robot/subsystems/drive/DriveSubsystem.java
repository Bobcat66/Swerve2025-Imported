package org.pihisamurai.frc2025.robot.subsystems.drive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.moduleTranslations;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants.ppConfig;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.MaxModuleSpeed;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.pihisamurai.frc2025.robot.Constants.Akit;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.TeleopDriveMode;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants.PIDControl;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.ReefFace;
import org.pihisamurai.frc2025.robot.commands.drive.DirectDriveToPose;
import org.pihisamurai.frc2025.robot.commands.drive.TeleopDriveCommand;
import org.pihisamurai.frc2025.robot.subsystems.Vision;
import org.pihisamurai.frc2025.robot.utils.Localization;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;

public class DriveSubsystem extends SubsystemBase {
    private final GyroIO gyroIO;
    private final Module[] modules = new Module[4];
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    }; //For delta tracking

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final Vision vision;
    public final DriveCommandFactory CommandBuilder;
    public DriveSubsystem(
        GyroIO gyroIO,
        ModuleIO FLModuleIO,
        ModuleIO FRModuleIO,
        ModuleIO RLModuleIO,
        ModuleIO RRModuleIO,
        Vision vision
    ){

        OdometryThread.getInstance().start();

        vision.registerMeasurementConsumer(poseEstimator::addVisionMeasurement);
        this.vision = vision;

        this.gyroIO = gyroIO;
        modules[0] = new Module(FLModuleIO,"FrontLeft");
        modules[1] = new Module(FRModuleIO,"FrontRight");
        modules[2] = new Module(RLModuleIO, "RearLeft");
        modules[3] = new Module(RRModuleIO, "RearRight");
        //DriveFeedforwards
        
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (ChassisSpeeds speeds, DriveFeedforwards ff) -> driveCO(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(
                    PIDControl.Trans.kP,
                    PIDControl.Trans.kI,
                    PIDControl.Trans.kD
                ),
                new PIDConstants(
                    PIDControl.Rot.kP,
                    PIDControl.Rot.kI,
                    PIDControl.Rot.kD
                )
            ),
            ppConfig,
            () -> {return false;},
            this
        );

        CommandBuilder = new DriveCommandFactory(this);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setModuleStates(SwerveModuleState[] states){
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void setModuleState(SwerveModuleState state, int moduleIndex) {
        modules[moduleIndex].setDesiredState(state);
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose(){
        Pose2d out = poseEstimator.getEstimatedPosition();
        return out;
    }

    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void resetPose(Pose2d newPose){
        poseEstimator.resetPosition(rawGyroRotation,getModulePositions(),newPose);
    }

    /**Field-oriented Closed-Loop driving*/
    public void driveFO(ChassisSpeeds speeds){
        //System.out.println("Driving field oriented " + speeds);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }
        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        if (Akit.enabled) {    
            Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
            Logger.recordOutput("SwerveStates/Actual", actualStates);
        }
    }

    /** Chassis-oriented Closed-loop driving */
    public void driveCO(ChassisSpeeds speeds) {
        //System.out.println("Driving Chassis Oriented " + speeds);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }
        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        if (Akit.enabled) {    
            Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
            Logger.recordOutput("SwerveStates/Actual", actualStates);
        }
    }

    /** Chassis-oriented Open-loop driving */
    public void driveOpenLoopCO(ChassisSpeeds speeds){
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredStateOpenLoop(setpointStates[i]);
        }
        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        if (Akit.enabled) {    
            Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
            Logger.recordOutput("SwerveStates/Actual", actualStates);
        }
    }

    /** Field-oriented Open-loop driving */
    public void driveOpenLoopFO(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredStateOpenLoop(setpointStates[i]);
        }
        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        if (Akit.enabled) {    
            Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
            Logger.recordOutput("SwerveStates/Actual", actualStates);
        }
    }

    @Override
    public void periodic(){
        
        //MUST BE CALLED BEFORE CONSUMING DATA FROM ODOMETRY THREAD
        OdometryThread.getInstance().poll();
        
        //Compile-time evaluation
        if (Akit.currentMode == 1){
            //This block will only compile when the code is running in a simulation
            gyroIO.updateFromKinematics(getModuleStates(), kinematics);
        }

        if (DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }
        }
        
        //Update gyro logging
        gyroIO.updateInputs(gyroInputs);

        if (Akit.enabled) {
            Logger.processInputs("Drive/Gyro", gyroInputs);
            Logger.recordOutput("Odometry/Robot",getPose());
        }

        //Update module logging, process odometry
        for (Module module : modules) {
            module.periodic();
        }

        //Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = OdometryThread.getInstance().sampleCount;
        for (int i = 0; i < sampleCount; i++){
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int modIndex = 0; modIndex < 4; modIndex++){
            
                modulePositions[modIndex] = modules[modIndex].getOdometryModulePositions()[i];
                moduleDeltas[modIndex] = new SwerveModulePosition(
                    modulePositions[modIndex].distanceMeters - lastModulePositions[modIndex].distanceMeters,
                    modulePositions[modIndex].angle);
                lastModulePositions[modIndex] = modulePositions[modIndex];
            }
            /*
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }*/
            poseEstimator.updateWithTime(sampleTimestamps[i],rawGyroRotation,modulePositions);
        }
        
        if (gyroInputs.connected) {
            rawGyroRotation = gyroInputs.yawPosition;
        } else {
            rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * 0.02));
        }

        poseEstimator.update(rawGyroRotation, getModulePositions());

        //Updates internal pose estimator with vision readings
        vision.updatePoseEstimator();
    }

    @Override
    public void simulationPeriodic(){
        
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Builds drive commands */
    public class DriveCommandFactory {

        private DriveSubsystem drive;
        private Map<TeleopDriveMode,Consumer<ChassisSpeeds>> driveModeMapping = new HashMap<>();

        private DriveCommandFactory(DriveSubsystem drive) {
            this.drive = drive;
            driveModeMapping.put(TeleopDriveMode.kClosedLoopFieldOriented,drive::driveFO);
            driveModeMapping.put(TeleopDriveMode.kClosedLoopChassisOriented,drive::driveCO);
            driveModeMapping.put(TeleopDriveMode.kOpenLoopFieldOriented,drive::driveOpenLoopFO);
            driveModeMapping.put(TeleopDriveMode.kOpenLoopChassisOriented,drive::driveOpenLoopCO);
        }

        public Command directDriveToPose(Pose2d targetPose) {
            return new DirectDriveToPose(drive, targetPose);
        }

        public Command directDriveToNearestBranch(boolean isLeftBranch, Transform2d offset) {
            return new Command() {

                Command driveCommand;
                boolean leftBranch;
                Transform2d robotOffset;

                {
                    this.leftBranch = isLeftBranch;
                    this.robotOffset = offset;
                    addRequirements(drive);
                }

                @Override 
                public void initialize() {
                    ReefFace closestFace = Localization.getClosestReefFace(drive.getPose());
                    Pose2d targetPose = leftBranch ? closestFace.leftBranch : closestFace.rightBranch;
                    targetPose.rotateBy(Rotation2d.fromDegrees(180));
                    targetPose = targetPose.transformBy(robotOffset);
                    driveCommand = directDriveToPose(targetPose);
                    driveCommand.initialize();
                }

                @Override
                public void execute() {
                    driveCommand.execute();
                }

                @Override
                public boolean isFinished() {
                    return driveCommand.isFinished();
                }

                @Override
                public void end(boolean interrupted) {
                    driveCommand.end(interrupted);
                }
            };
        }

        /** Generates a teleop drive command. Set headingSupplier to null to unlock heading */

        public TeleopDriveCommand driveTeleop(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, TeleopDriveMode mode, double transClutchFactor, double rotClutchFactor, Supplier<Rotation2d> headingSupplier) {
            TeleopDriveCommand driveCommand = new TeleopDriveCommand(drive, driveModeMapping, xSupplier, ySupplier, omegaSupplier, mode);
            driveCommand.setClutchFactors(transClutchFactor, rotClutchFactor);
            driveCommand.lockHeading(headingSupplier);
            return driveCommand;
        }
    }
    
}
