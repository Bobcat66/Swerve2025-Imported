package org.pihisamurai.frc2025.robot.commands.drive;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.apache.commons.lang3.function.TriFunction;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.AutoConstants;
import org.pihisamurai.frc2025.robot.Constants.DriveConstants.TeleopDriveMode;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PointOfInterest;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PoseOfInterest;
import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.doubleClutchRotationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.doubleClutchTranslationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.maxRotationSpeed;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.maxTranslationSpeed;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.singleClutchRotationFactor;
import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.singleClutchTranslationFactor;

public class TeleopDriveCommand extends Command {

    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawOmegaSupplier;
    private final Map<TeleopDriveMode,Consumer<ChassisSpeeds>> driveModeMapping;
    private final DriveSubsystem drive;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;
    private Consumer<ChassisSpeeds> driveConsumer;

    private PIDController turnController;
    
    //Command control variables
    private double transClutchFactor = 1.0;
    private double rotClutchFactor = 1.0;
    private Supplier<Rotation2d> headingSupplier = null;
    private TeleopDriveMode mode;
    
    public TeleopDriveCommand(DriveSubsystem drive, Map<TeleopDriveMode,Consumer<ChassisSpeeds>> driveModeMapping,DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, TeleopDriveMode mode) {
        this.rawXSupplier = () -> xSupplier.getAsDouble() * maxTranslationSpeed;
        this.rawYSupplier = () -> ySupplier.getAsDouble() * maxTranslationSpeed;
        this.rawOmegaSupplier = () -> omegaSupplier.getAsDouble() * maxRotationSpeed;
        this.driveModeMapping = driveModeMapping;
        this.drive = drive;
        this.mode = mode;
        this.turnController = new PIDController(
            AutoConstants.PIDControl.Rot.kP,
            AutoConstants.PIDControl.Rot.kI,
            AutoConstants.PIDControl.Rot.kD
        );
        this.turnController.enableContinuousInput(-Math.PI, Math.PI);
        reloadCommand();
        addRequirements(drive);
    }

    /** Updates command with new values */
    private void reloadCommand() {
        xSupplier = () -> rawXSupplier.getAsDouble() * transClutchFactor;
        ySupplier = () -> rawYSupplier.getAsDouble() * transClutchFactor;
        omegaSupplier = (headingSupplier == null) 
            ? () -> rawOmegaSupplier.getAsDouble() * rotClutchFactor
            : () -> turnController.calculate(drive.getHeading().getRadians(),headingSupplier.get().getRadians());
        driveConsumer = driveModeMapping.get(mode);
    }

    @Override
    public void execute(){
        ChassisSpeeds speeds = new ChassisSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            omegaSupplier.getAsDouble()
        );
        driveConsumer.accept(speeds);
    }

    //Automatically cancels drive command when teleop is not enabled
    @Override
    public boolean isFinished(){
        return !DriverStation.isTeleopEnabled();
    }

    public void lockHeading(Rotation2d heading) {
        headingSupplier = () -> heading;
        reloadCommand();
    }

    public void lockHeading(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = headingSupplier;
        reloadCommand();
    }

    public void unlockHeading() {
        headingSupplier = null;
        reloadCommand();
    }

    public void setClutchFactors(double transFactor, double rotFactor) {
        this.transClutchFactor = transFactor;
        this.rotClutchFactor = rotFactor;
        reloadCommand();
    }

    //** Returns a command that applies a heading lock to the TeleopDriveCommand */
    public Command applyHeadingLock(Supplier<Rotation2d> headingSupplier){
        return Commands.startEnd(() -> this.lockHeading(headingSupplier),() -> this.unlockHeading());
    }

    //** Returns a command that applies a heading lock to the TeleopDriveCommand */
    public Command applyHeadingLock(Rotation2d heading){
        return Commands.startEnd(() -> this.lockHeading(heading),() -> this.unlockHeading());
    }

    //** Returns a command that applies a clutch to the TeleopDriveCommand */
    public Command applyClutchFactors(double transFactor, double rotFactor){
        return Commands.startEnd(() -> this.setClutchFactors(transFactor, rotFactor), () -> this.setClutchFactors(1.0, 1.0));
    }

    //** Returns a command that applies a single clutch to the TeleopDriveCommand */
    public Command applySingleClutch() {
        return applyClutchFactors(singleClutchTranslationFactor, singleClutchRotationFactor);
    }

    //** Returns a command that applies a double clutch to the TeleopDriveCommand */
    public Command applyDoubleClutch() {
        return applyClutchFactors(doubleClutchTranslationFactor, doubleClutchRotationFactor);
    }

    //** Returns a command that applies a heading lock orentied to the center of the reef to the TeleopDriveCommand */
    public Command applyReefHeadingLock(){
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue 
                ? () -> {
                    Translation2d teamReef = PointOfInterest.BLU_REEF.position;
                    Rotation2d angleToReef = teamReef.minus(drive.getPose().getTranslation()).getAngle();
                    return angleToReef.rotateBy(Rotation2d.fromDegrees(180));
                }
                : () -> {
                    Translation2d teamReef = PointOfInterest.RED_REEF.position;
                    Rotation2d angleToReef = teamReef.minus(drive.getPose().getTranslation()).getAngle();
                    return angleToReef.rotateBy(Rotation2d.fromDegrees(180));
                }
        );
    }

    //** Returns a command that applies a heading lock oriented to the processor-side coral station to the TeleopDriveCommand */
    public Command applyProcessorCoralStationHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue
                ? PoseOfInterest.BLU_RIGHT_STATION.pose.getRotation()
                : PoseOfInterest.RED_RIGHT_STATION.pose.getRotation()
        );
    }

    //** Returns a command that applies a heading lock oriented to the opposite-side coral station to the TeleopDriveCommand */
    public Command applyOppositeCoralStationHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue
                ? PoseOfInterest.BLU_LEFT_STATION.pose.getRotation()
                : PoseOfInterest.RED_LEFT_STATION.pose.getRotation()
        );
    }

    //** Returns a command that applies a heading lock oriented directly forward to the TeleopDriveCommand */
    public Command applyForwardHeadingLock() {
        return applyHeadingLock(
            DriverStation.getAlliance().get() == Alliance.Blue
                ? Rotation2d.fromDegrees(0)
                : Rotation2d.fromDegrees(180)
        );
    }
}
