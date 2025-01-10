package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class GyroIOSim implements GyroIO{
    
    private Rotation2d simRotation = new Rotation2d();
   
    public GyroIOSim() {

    }

    //initially updates the position once the gyro is connected
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = simRotation;
        inputs.yawVelocityDegPerSec = 0.0; //Different from last year's code, check if works
    }

    //calculates how much robot has rotated and adds it to where it previously was oriented
    @Override
    public void deriveGyro(SwerveModuleState[] swerveModuleStates, SwerveDriveKinematics kinematics){
        simRotation = simRotation.plus(Rotation2d.fromRadians(kinematics.toChassisSpeeds(swerveModuleStates).omegaRadiansPerSecond *0.02));
        Logger.recordOutput("TEST/swerveModuleStates", swerveModuleStates);
        Logger.recordOutput("TEST/simRotation", simRotation);
    }

    @Override
    public void resetHeading() {

    }    
}
