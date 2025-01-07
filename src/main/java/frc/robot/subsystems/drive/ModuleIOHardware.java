package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Turn;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.utils.SignalUtils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.stream.Stream;

import static frc.robot.Constants.DriveConstants.odometryFrequencyHz;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

/*Designed for Bebop (Revrobotics SparkMAX + CTRE CANcoder)*/
public class ModuleIOHardware implements ModuleIO {
    
    private final SparkMax m_turnMotor;
    private final SparkClosedLoopController TurnPID;
    private final RelativeEncoder TurnRelEncoder;

    private final SparkMax m_driveMotor;
    private final SparkClosedLoopController DrivePID;
    private final RelativeEncoder DriveRelEncoder;

    private final BlockingQueue<Long> timestampQueue;
    private final BlockingQueue<Double> drivePositionQueue;
    private final BlockingQueue<Double> turnPositionQueue;

    private final ArrayList<Long> timestampBuffer = new ArrayList<>();
    private final ArrayList<Double> drivePositionBuffer = new ArrayList<>();
    private final ArrayList<Double> turnPositionBuffer = new ArrayList<>();

    private final CANcoder m_turnEncoder;
    private final StatusSignal<Angle> turnAbsolutePosition;

    public ModuleIOHardware(ModuleConfig config){

        m_turnMotor = new SparkMax(config.TurnPort,MotorType.kBrushless);
        TurnPID = m_turnMotor.getClosedLoopController();
        TurnRelEncoder = m_turnMotor.getEncoder();

        m_driveMotor = new SparkMax(config.DrivePort,MotorType.kBrushless);
        DrivePID = m_driveMotor.getClosedLoopController();
        DriveRelEncoder = m_driveMotor.getEncoder();
    
        m_turnEncoder = new CANcoder(config.EncoderPort);

        //Config turn absolute encoder here
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.FutureProofConfigs = false;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderConfig.MagnetSensor.MagnetOffset = config.EncoderOffsetRots;
        m_turnEncoder.getConfigurator().apply(encoderConfig);
        turnAbsolutePosition = m_turnEncoder.getAbsolutePosition();

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
            .primaryEncoderPositionPeriodMs((int) (1000/odometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_turnMotor.configure(turnConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        TurnRelEncoder.setPosition(turnAbsolutePosition.getValueAsDouble());
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
            .primaryEncoderPositionPeriodMs((int) (1000/odometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        DriveRelEncoder.setPosition(0.0);
        
        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(SignalUtils.getSparkMaxPositionSignal(m_driveMotor));
        OdometryThread.getInstance().registerErrorSignal(SignalUtils.getSparkMaxErrorSignal(m_driveMotor));
        turnPositionQueue = OdometryThread.getInstance().registerSignal(SignalUtils.getSparkMaxPositionSignal(m_turnMotor));
        OdometryThread.getInstance().registerErrorSignal(SignalUtils.getSparkMaxErrorSignal(m_turnMotor));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){

        inputs.drivePositionMeters = DriveRelEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = DriveRelEncoder.getVelocity();
        inputs.driveAppliedVolts = m_driveMotor.getBusVoltage() * m_driveMotor.getAppliedOutput();
        inputs.driveCurrentAmps = m_driveMotor.getOutputCurrent();
        
        inputs.turnAbsolutePosition = new Rotation2d(turnAbsolutePosition.refresh().getValue());
        inputs.turnPosition = Rotation2d.fromRotations(TurnRelEncoder.getPosition());
        inputs.turnVelocityRPM = TurnRelEncoder.getVelocity();
        inputs.turnAppliedVolts = m_turnMotor.getBusVoltage() * m_turnMotor.getAppliedOutput();
        inputs.driveCurrentAmps = m_turnMotor.getOutputCurrent();

        /** Should be called after poll() in the main thread */
        int samples = OdometryThread.getInstance().sampleCount;
        OdometryThread.safeDrain(timestampQueue, timestampBuffer,samples);
        OdometryThread.safeDrain(drivePositionQueue, drivePositionBuffer, samples);
        OdometryThread.safeDrain(turnPositionQueue, turnPositionBuffer, samples);
        inputs.odometryTimestamps = timestampBuffer.stream().mapToDouble((Long value) -> value/1e6).toArray();
        inputs.odometryDrivePositionsMeters = drivePositionBuffer.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionBuffer.stream().map((Double value) -> Rotation2d.fromRotations(value)).toArray(Rotation2d[]::new);
        timestampBuffer.clear();
        drivePositionBuffer.clear();
        turnPositionBuffer.clear();
    }

    @Override
    public void setDriveVolts(double volts){
        m_driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVolts(double volts){
        m_turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSec, double FFVolts){
        DrivePID.setReference(
            velocityMetersPerSec,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            FFVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(double positionRots, double FFVolts){
        TurnPID.setReference(
            positionRots,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            FFVolts,
            ArbFFUnits.kVoltage
        );
    }
    
}
