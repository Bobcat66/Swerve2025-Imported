package frc.robot.subsystems.drive;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import static frc.robot.Constants.DriveConstants.GyroConstants.kGyroPort;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.Constants.DriveConstants.odometryFrequencyHz;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

/*Designed to work with CTRE Pigeon 2*/
public class GyroIOHardware implements GyroIO {
    private final Pigeon2 m_gyro = new Pigeon2(kGyroPort);
    private final StatusSignal<Angle> yaw = m_gyro.getYaw();
    private final ConcurrentLinkedQueue<Double> yawPositionQueue;
    private final ConcurrentLinkedQueue<Long> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = m_gyro.getAngularVelocityZWorld();

    private final ArrayList<Double> yawPositionBuffer = new ArrayList<>(20);
    private final ArrayList<Long> yawTimestampBuffer = new ArrayList<>(20);
    
    public GyroIOHardware() {
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(odometryFrequencyHz);
        yawVelocity.setUpdateFrequency(50);
        //m_gyro.optimizeBusUtilization();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(() -> yaw.getValue().in(Degrees));
        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = BaseStatusSignal.refreshAll(yaw,yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = new Rotation2d(yaw.getValue());
        inputs.yawVelocityDegPerSec = yawVelocity.getValue().in(DegreesPerSecond);

        int samples = OdometryThread.getInstance().sampleCount;
        OdometryThread.safeDrain(yawTimestampQueue, yawTimestampBuffer,samples);
        OdometryThread.safeDrain(yawPositionQueue, yawPositionBuffer, samples);
        inputs.odometryYawTimestamps = yawTimestampBuffer.stream().mapToDouble((Long value) -> value/1e6).toArray();
        inputs.odometryYawPositions = yawPositionBuffer.stream().map((Double value) -> Rotation2d.fromDegrees(value)).toArray(Rotation2d[]::new);
    }

}
