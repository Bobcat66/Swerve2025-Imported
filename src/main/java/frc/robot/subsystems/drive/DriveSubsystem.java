package frc.robot.subsystems.drive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import frc.robot.Constants.DriveConstants.AutoConstants.PIDControl;
import static frc.robot.Constants.DriveConstants.moduleTranslations;
import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.MaxModuleSpeed;
import static frc.robot.Constants.DriveConstants.AutoConstants.ppConfig;

public class DriveSubsystem extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
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

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    public DriveSubsystem(
        GyroIO gyroIO,
        ModuleIO FLModuleIO,
        ModuleIO FRModuleIO,
        ModuleIO RLModuleIO,
        ModuleIO RRModuleIO
    ){
        this.gyroIO = gyroIO;
        modules[0] = new Module(FLModuleIO,"FrontLeft");
        modules[1] = new Module(FRModuleIO,"FrontRight");
        modules[2] = new Module(RLModuleIO, "RearLeft");
        modules[3] = new Module(RRModuleIO, "RearRight");

        OdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            (ChassisSpeeds speeds, DriveFeedforwards ff) -> driveCLCO(speeds), //TODO: Implement drivefeedforwards
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
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose){
        poseEstimator.resetPosition(rawGyroRotation,getModulePositions(),newPose);
    }

    /**Chassis-oriented Closed-Loop driving*/
    public void driveCLCO(ChassisSpeeds speeds){
        ChassisSpeeds discSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MaxModuleSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(setpointStates[i]);
        }
        Logger.recordOutput("SwerveStates/Setpoints",setpointStates);
    }

    /**Field-oriented Closed-loop driving */
    public void driveCLFO(ChassisSpeeds speeds){

    }

    @Override
    public void periodic(){
        DriveSubsystem.odometryLock.lock();
        try{
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Drive/Gyro", gyroInputs);

            for (Module module : modules) {
                module.periodic();
            }

            if (DriverStation.isDisabled()) {
                for (Module module : modules) {
                    module.stop();
                }
            }

            //Update odometry
            double[] sampleTimestamps = modules[0].getOdometryTimestamps();
            int sampleCount = sampleTimestamps.length;
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
                if (gyroInputs.connected) {
                    // Use the real gyro angle
                    rawGyroRotation = gyroInputs.odometryYawPositions[i];
                } else {
                    // Use the angle delta from the kinematics and module deltas
                    Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                    rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
                }
                poseEstimator.updateWithTime(sampleTimestamps[i],rawGyroRotation,modulePositions);
            }
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
        rawGyroRotation = gyroInputs.yawPosition;
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
