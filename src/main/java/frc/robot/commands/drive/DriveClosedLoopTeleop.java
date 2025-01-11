package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
/**
 * Drives Teleop in closed loop mode from controller inputs
 */
public class DriveClosedLoopTeleop extends Command {
    private final DriveSubsystem m_subsystem;
    private final DoubleSupplier xTransSpeedSupplier; // field-oriented x translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier yTransSpeedSupplier; // field-oriented y translational speed, scaled from -1.0 to 1.0
    private final DoubleSupplier omegaSupplier; // rotational speed, scaled from -1.0 to 1.0
    private final BooleanSupplier singleClutchSupplier;
    private final BooleanSupplier doubleClutchSupplier;
    
    public DriveClosedLoopTeleop(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier,
        BooleanSupplier singleClutchSupplier,
        BooleanSupplier doubleClutchSupplier,
        DriveSubsystem subsystem) {
        this.xTransSpeedSupplier = xSupplier;
        this.yTransSpeedSupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.singleClutchSupplier = singleClutchSupplier;
        this.doubleClutchSupplier = doubleClutchSupplier;
        this.m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = new ChassisSpeeds(
            applyTranslationClutch(scaleSpeed(xTransSpeedSupplier.getAsDouble()) * DriveConstants.maxTranslationSpeed),
            applyTranslationClutch(scaleSpeed(yTransSpeedSupplier.getAsDouble()) * DriveConstants.maxTranslationSpeed),
            applyRotationClutch(omegaSupplier.getAsDouble()  * DriveConstants.maxRotationSpeed)
        );
        m_subsystem.drive(
            speeds
        );
    }

    @Override
    public void end(boolean interrupted) {
        //m_subsystem.stop();
    }

    public double scaleSpeed(double speed){
        return speed / Math.max(Math.sqrt(Math.pow(xTransSpeedSupplier.getAsDouble(), 2) + Math.pow(yTransSpeedSupplier.getAsDouble(), 2)), 1);
    }

    public double applyTranslationClutch(double translationalSpeed){
        if(doubleClutchSupplier.getAsBoolean()) return translationalSpeed * DriveConstants.doubleClutchTranslationFactor;
        if(singleClutchSupplier.getAsBoolean()) return translationalSpeed * DriveConstants.singleClutchTranslationFactor;
        return translationalSpeed;
    }

    public double applyRotationClutch(double rotationalSpeed){
        if(doubleClutchSupplier.getAsBoolean()) return rotationalSpeed * DriveConstants.doubleClutchRotationFactor;
        if(singleClutchSupplier.getAsBoolean()) return rotationalSpeed * DriveConstants.singleClutchRotationFactor;
        return rotationalSpeed;
    }
}
