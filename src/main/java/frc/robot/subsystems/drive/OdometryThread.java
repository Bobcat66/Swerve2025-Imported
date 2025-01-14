package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.Akit;

import static frc.robot.Constants.DriveConstants.odometryFrequencyHz;

/**An odometry thread designed to work with SparkMax motor controllers. This is a singleton class, only one instance exists at any given time*/
public class OdometryThread {

    //SHARED RESOURCES
    private final ArrayList<DoubleSupplier> signals = new ArrayList<>();
    private final ArrayList<BooleanSupplier> errorSignals = new ArrayList<>();

    private final ArrayList<ConcurrentLinkedQueue<Double>> signalQueues = new ArrayList<>();
    private final ArrayList<ConcurrentLinkedQueue<Long>> timestampQueues = new ArrayList<>();

    private final Lock signalLock = new ReentrantLock();

    private static OdometryThread instance = null;

    private Notifier notifier = new Notifier(this::run);

    //ODOMETRY THREAD ONLY
    private AtomicInteger samplesSinceLastPoll = new AtomicInteger(0);

    //MAIN THREAD ONLY
    public int sampleCount = 0;

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

        //Compile-time evaluation
        if (Akit.currentMode == 1) {
            //Running simulation, disable odometry thread
            sampleCount = 1;
            return;
        }
        
        notifier.startPeriodic(1.0/odometryFrequencyHz);
    }

    /**Registers a signal from the main thread*/
    public ConcurrentLinkedQueue<Double> registerSignal(DoubleSupplier signal){
        ConcurrentLinkedQueue<Double> queue = new ConcurrentLinkedQueue<>();
        synchronized (signalLock) {
            signals.add(signal);
            signalQueues.add(queue);
        }
        return queue;
    }

    /**Registers an error signal from the main thread. If any error signals are detected, then the thread does not register odometry for any devices during the given odometry cycle */
    public void registerErrorSignal(BooleanSupplier errorSignal){
        synchronized (signalLock) {
            errorSignals.add(errorSignal);
        }
    }

    /**Makes a timestamp queue. Timestamps are recorded in microseconds as a Long */
    public ConcurrentLinkedQueue<Long> makeTimestampQueue(){
        ConcurrentLinkedQueue<Long> queue = new ConcurrentLinkedQueue<>();
        synchronized (signalLock) {
            timestampQueues.add(queue);
        }
        return queue;
    }

    /**Periodic function to run in the odometry thread, updates queues with latest odometry values*/
    private void run(){

        long timestamp = RobotController.getFPGATime();

        for (int i = 0; i < errorSignals.size(); i++){
            if (errorSignals.get(i).getAsBoolean()){
                return;
            }
        }

        synchronized (signalLock) {
            for (int i = 0; i < signals.size(); i++){
                signalQueues.get(i).offer(signals.get(i).getAsDouble());
            }
            for (int i = 0; i < timestampQueues.size(); i++){
                timestampQueues.get(i).offer(timestamp);
            }
        }
        samplesSinceLastPoll.incrementAndGet(); //All queues are GUARANTEED to have at least samplesSinceLastPoll elements in them, all correctly ordered

    }

    /** This method should be called ONCE per main-cycle thread */
    public void poll() {
        if (Akit.currentMode == 1){
            //Disables polling during simulation
            return;
        }
        sampleCount = samplesSinceLastPoll.getAndSet(0);
    }

    /** Safely reads N elements of type T from a queue into a collection. Will not block if queue is empty*/
    public static <T> void safeDrain(ConcurrentLinkedQueue<T> source, Collection<T> dest, int n) {
        for (int i = 0; i < n; i++){
            dest.add(source.poll());
        }
    }
    
}
