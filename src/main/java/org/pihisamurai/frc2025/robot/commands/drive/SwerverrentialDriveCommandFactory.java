package org.pihisamurai.frc2025.robot.commands.drive;

import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.maxTranslationSpeed;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SwerverrentialDriveCommandFactory {
    private final DriveSubsystem m_swerveDrive;
    private final DifferentialDrive diffDrive;
    private final DoubleConsumer leftSpeedConsumer;
    private final DoubleConsumer rightSpeedConsumer;
    public SwerverrentialDriveCommandFactory(DriveSubsystem drive) {
        m_swerveDrive = drive;
        leftSpeedConsumer = (dutyCycle) -> {
            m_swerveDrive.setModuleState(new SwerveModuleState(dutyCycle * maxTranslationSpeed,Rotation2d.kZero),0);
            m_swerveDrive.setModuleState(new SwerveModuleState(dutyCycle * maxTranslationSpeed,Rotation2d.kZero),2);
        };
        
        rightSpeedConsumer = (dutyCycle) -> {
            m_swerveDrive.setModuleState(new SwerveModuleState(dutyCycle * maxTranslationSpeed,Rotation2d.kZero),1);
            m_swerveDrive.setModuleState(new SwerveModuleState(dutyCycle * maxTranslationSpeed,Rotation2d.kZero),3);
        };
        diffDrive = new DifferentialDrive(leftSpeedConsumer, rightSpeedConsumer);
    }

    public Command buildTankDriveCommand(DoubleSupplier leftStick, DoubleSupplier rightStick) {
        return Commands.run(
            () -> diffDrive.tankDrive(leftStick.getAsDouble(), rightStick.getAsDouble()),
            m_swerveDrive
        );
    }

    public Command buildArcadeDriveCommand(DoubleSupplier speedStick, DoubleSupplier turnStick) {
        return Commands.run(
            () -> diffDrive.arcadeDrive(speedStick.getAsDouble(), turnStick.getAsDouble()),
            m_swerveDrive
        );
    }

    public Command buildCurvatureDriveCommand(DoubleSupplier speedStick, DoubleSupplier turnStick, boolean allowTurnInPlace) {
        return Commands.run(
            () -> diffDrive.curvatureDrive(speedStick.getAsDouble(), turnStick.getAsDouble(), allowTurnInPlace), 
            m_swerveDrive
        );
    }
}
