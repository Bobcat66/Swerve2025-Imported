package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double xAccel = 0.0;
        public double yAccel = 0.0;
        public double yawVelocityDegPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    /** Updates inputs with regular data */
    public abstract void updateInputs(GyroIOInputs inputs);

    public default void deriveGyro(SwerveModuleState[] swerveModuleState, SwerveDriveKinematics kinematics) {}

    public default void resetHeading() {} 
}
