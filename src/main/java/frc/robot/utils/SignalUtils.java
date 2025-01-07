package frc.robot.utils;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;

public class SignalUtils {
    /**Returns a spark max error signal. WARNING: NOT THREAD-SAFE, LOCKING MUST BE IMPLEMENTED IN THE CALLING THREAD TO PREVENT RACE CONDITIONS*/
    public static BooleanSupplier getSparkMaxErrorSignal(SparkMax motor){
        return () -> (motor.getLastError() != REVLibError.kOk);
    }

    /**Returns a spark max position signal. WARNING: NOT THREAD-SAFE */
    public static DoubleSupplier getSparkMaxPositionSignal(SparkMax motor){
        return () -> (motor.getEncoder().getPosition());
    }
}
