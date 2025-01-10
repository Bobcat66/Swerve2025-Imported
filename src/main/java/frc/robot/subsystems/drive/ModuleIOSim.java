package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.odometryFrequencyHz;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Turn;
import frc.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;

public class ModuleIOSim implements ModuleIO {
    private final DCMotor neoSim = DCMotor.getNEO(1);

    private final SparkMax m_turnMotor;
    private final SparkMaxSim m_turnSim;
    private final SparkRelativeEncoderSim turnEncoderSim;
    private final SparkClosedLoopController turnController;
    private final ClosedLoopSlot turnSimSlot;

    private final SparkMax m_driveMotor;
    private final SparkMaxSim m_driveSim;
    private final SparkRelativeEncoderSim driveEncoderSim;
    private final SparkClosedLoopController driveController;
    private final ClosedLoopSlot driveSimSlot;
    
    public ModuleIOSim(ModuleConfig config){
        
        m_turnMotor = new SparkMax(config.TurnPort,MotorType.kBrushless);
        m_turnSim = new SparkMaxSim(m_turnMotor, neoSim);
        turnController = m_turnMotor.getClosedLoopController();
        turnEncoderSim = m_turnSim.getRelativeEncoderSim();
        turnSimSlot = m_turnSim.getClosedLoopSlot();

        m_driveMotor = new SparkMax(config.DrivePort,MotorType.kBrushless);
        m_driveSim = new SparkMaxSim(m_driveMotor, neoSim);
        driveController = m_driveMotor.getClosedLoopController();
        driveEncoderSim = m_driveSim.getRelativeEncoderSim();
        driveSimSlot = m_driveSim.getClosedLoopSlot();

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Turn.CurrentLimit)
            .voltageCompensation(Turn.VoltageCompensation);
        turnConfig
            .encoder
            .positionConversionFactor(Turn.PositionConversionFactor)
            .velocityConversionFactor(Turn.VelocityConversionFactor);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Turn.kP)
            .i(Turn.kI)
            .d(Turn.kD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0,1);
        turnConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_turnMotor.configure(turnConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drive.CurrentLimit)
            .voltageCompensation(Drive.VoltageCompensation);
        driveConfig
            .encoder
            .positionConversionFactor(Drive.PositionConversionFactor)
            .velocityConversionFactor(Drive.VelocityConversionFactor);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Drive.kP)
            .i(Drive.kI)
            .d(Drive.kD);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveEncoderSim.getPosition();
        inputs.driveVelocityMetersPerSec = driveEncoderSim.getVelocity();
        inputs.driveAppliedVolts = m_driveSim.getBusVoltage() * m_driveSim.getAppliedOutput();
        inputs.driveCurrentAmps = m_driveSim.getMotorCurrent();
        
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnEncoderSim.getPosition());
        inputs.turnPosition = Rotation2d.fromRotations(turnEncoderSim.getPosition());
        inputs.turnVelocityRPM = turnEncoderSim.getVelocity();
        inputs.turnAppliedVolts = m_turnSim.getBusVoltage() * m_turnSim.getAppliedOutput();
        inputs.driveCurrentAmps = m_turnSim.getMotorCurrent();

        inputs.odometryTimestamps = new double[]{RobotController.getFPGATime()/1000000.0};
        inputs.odometryDrivePositionsMeters = new double[]{driveEncoderSim.getPosition()};
        inputs.odometryTurnPositions = new Rotation2d[]{Rotation2d.fromRotations(turnEncoderSim.getPosition())};

    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec, double FFVolts) {
        driveController.setReference(
            velocityMetersPerSec,
            ControlType.kVelocity,
            driveSimSlot,
            FFVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(double positionRots, double FFVolts){
        turnController.setReference(
            positionRots,
            ControlType.kPosition,
            turnSimSlot,
            FFVolts,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setDriveVolts(double volts){
        m_driveSim.setAppliedOutput(volts/m_driveSim.getBusVoltage());
    }

    @Override
    public void setTurnVolts(double volts){
        m_turnSim.setAppliedOutput(volts/m_turnSim.getBusVoltage());
    }
    
}
