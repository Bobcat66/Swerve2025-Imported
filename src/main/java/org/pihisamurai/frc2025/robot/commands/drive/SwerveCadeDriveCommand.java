package org.pihisamurai.frc2025.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.pihisamurai.frc2025.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;

/** Takes a swerve drive and makes it worse by making it emulate an arcade drive */
public class SwerveCadeDriveCommand extends Command {
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier turnSupplier;
    private final DriveSubsystem m_drive;
    private final DifferentialDrive diffDrive;
    public SwerveCadeDriveCommand(DriveSubsystem drive, DoubleSupplier speed, DoubleSupplier turn) {
        m_drive = drive;
        speedSupplier = speed;
        turnSupplier = turn;
    }
    
    @Override
    public void initialize() {
        SwerveModuleState[] straightWheels = {
            new SwerveModuleState(0,Rotation2d.kZero),
            new SwerveModuleState(0,Rotation2d.kZero),
            new SwerveModuleState(0,Rotation2d.kZero),
            new SwerveModuleState(0,Rotation2d.kZero)
        };
        m_drive.setModuleStates(straightWheels);
    }

    @Override
    public void execute() {
        
    }

}
