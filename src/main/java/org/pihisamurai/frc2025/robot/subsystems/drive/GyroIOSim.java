package org.pihisamurai.frc2025.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

public class GyroIOSim implements GyroIO {

    private Rotation2d simRotation = new Rotation2d();
    private double velocityRadsPerSec = 0;
   
    public GyroIOSim() {}

    //initially updates the position once the gyro is connected
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = simRotation;
        inputs.yawVelocityDegPerSec = velocityRadsPerSec * (180/Math.PI);
        inputs.odometryYawPositions = new Rotation2d[] {simRotation};
        inputs.odometryYawTimestamps = new double[] {RobotController.getFPGATime()/1000000.0};
    }

    //calculates how much robot has rotated and adds it to where it previously was oriented
    public void updateFromKinematics(SwerveModuleState[] swerveModuleStates, SwerveDriveKinematics kinematics){
        velocityRadsPerSec = kinematics.toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond;
        simRotation = simRotation.plus(Rotation2d.fromRadians(velocityRadsPerSec * 0.02));
    }

    @Override
    public void resetHeading() {
        
    }    
}
