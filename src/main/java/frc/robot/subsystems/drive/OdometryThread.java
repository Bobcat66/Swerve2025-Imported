package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

import static frc.robot.Constants.DriveConstants.odometryFrequencyHz;

/**An odometry thread designed to work with SparkMax motor controllers. This is a singleton class, only one instance exists at any given time*/
public class OdometryThread {
    //public static record odomReading(double val,long timestamp) {}
    private final ArrayList<DoubleSupplier> signals = new ArrayList<>();
    private final ArrayList<BooleanSupplier> errorSignals = new ArrayList<>();

    private final ArrayList<Queue<Double>> signalQueues = new ArrayList<>();
    private final ArrayList<Queue<Long>> timestampQueues = new ArrayList<>();
    //TODO: Make struct array

    private static OdometryThread instance = null;

    private Notifier notifier = new Notifier(this::run);

    private OdometryThread(){
        notifier.setName("OdometryThread");
    }

    public static OdometryThread getInstance() {
        if (instance == null){
            instance = new OdometryThread();
        }
        return instance;
    }

    public void start(){
        notifier.startPeriodic(1.0/odometryFrequencyHz);
    }

    /**Registers a signal from the main thread*/
    public Queue<Double> registerSignal(DoubleSupplier signal){
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        DriveSubsystem.odometryLock.lock();
        try {
            signals.add(signal);
            signalQueues.add(queue);
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    /**Registers an error signal from the main thread. If any error signals are detected, then the thread does not register odometry for any devices during the given odometry cycle */
    public void registerErrorSignal(BooleanSupplier errorSignal){
        DriveSubsystem.odometryLock.lock();
        try {
            errorSignals.add(errorSignal);
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
    }
    /**Makes a timestamp queue. Timestamps are recorded in microseconds as a Long */
    public Queue<Long> makeTimestampQueue(){
        Queue<Long> queue = new ArrayBlockingQueue<>(20);
        DriveSubsystem.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
        return queue;
    }
    /**Periodic function to run in the odometry thread, updates queues with latest odometry values*/
    private void run(){
        DriveSubsystem.odometryLock.lock();
        try {
            long timestamp = RobotController.getFPGATime();
            boolean isValid = true;
            for (int i = 0; i < errorSignals.size(); i++){
                if (errorSignals.get(i).getAsBoolean()){
                    isValid = false;
                    break;
                }
            }
            
            if (isValid) {
                for (int i = 0; i < signals.size(); i++){
                    signalQueues.get(i).offer(signals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++){
                    timestampQueues.get(i).offer(timestamp);
                }
            }
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
    }
    
}
