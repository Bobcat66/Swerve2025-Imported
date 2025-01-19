package org.pihisamurai.frc2025.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityDegPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public abstract void updateInputs(GyroIOInputs inputs);

    public default void resetHeading() {} 

    /** Updates IOLayer's internal state from kinematics (FOR SIM ONLY)*/
    public default void updateFromKinematics(SwerveModuleState[] swerveModuleStates, SwerveDriveKinematics kinematics) {}
}
