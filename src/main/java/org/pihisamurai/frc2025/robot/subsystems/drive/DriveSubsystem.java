package org.pihisamurai.frc2025.robot.subsystems.drive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.moduleTranslations;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants.ppConfig;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.MaxModuleSpeed;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.pihisamurai.frc2025.robot.Constants.Akit;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants.PIDControl;
import org.pihisamurai.frc2025.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
            () -> {System.out.println("getPose called by pp");Pose2d output = getPose();System.out.println("Pose retrieved");return output;},//this::getPose,
            (pose) -> {System.out.println("resetPose called by pp");resetPose(pose);},
            () -> {System.out.println("getChassisSpeeds called by pp");ChassisSpeeds speeds = getChassisSpeeds();System.out.println("Speeds retrieved");return speeds;},
            (ChassisSpeeds speeds, DriveFeedforwards ff) -> {System.out.println("driveCO called by pp");driveCO(speeds);},
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
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++){
            states[i] = modules[i].getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose(){
        Pose2d out = poseEstimator.getEstimatedPosition();
        //System.out.println(out);
        return out;
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
        Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
        Logger.recordOutput("SwerveStates/Actual", actualStates);
    }

    /** Chassis-oriented Closed-loop driving */
    public void driveCO(ChassisSpeeds speeds) {
        System.out.println("Driving Chassis Oriented " + speeds);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02));
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        System.out.println("Desaturated wheel speeds: " + setpointStates[1]);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }
        SwerveModuleState[] actualStates = {modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState()};
        Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
        Logger.recordOutput("SwerveStates/Actual", actualStates);
        //System.out.println("Finished Driving Chassis Oriented");
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
        Logger.processInputs("Drive/Gyro", gyroInputs);

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
}
