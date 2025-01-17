// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.instrument.Instrumentation;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.function.Supplier;

import debug.DebugInstrumentator;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
    * Main initialization function. Do not perform any initialization here.
    *
    * <p>If you change your main robot class, change the parameter type.
    */
    public static void main(String... args) {
        DebugInstrumentator.getInstance().testInstrumentation();
        DebugInstrumentator.getInstance().load();
        System.out.println("INSTRUMENTATION LOADED");
        System.out.println(DebugInstrumentator.getInstance().classLoader);
        try{ 
            Class<?> robotBaseClass = DebugInstrumentator.getInstance().classLoader.loadClass("edu.wpi.first.wpilibj.RobotBase");
            Class<?> robotClass = DebugInstrumentator.getInstance().classLoader.loadClass("frc.robot.Robot");
            Supplier<Robot> robotConstructor = () -> {try {return (Robot)robotClass.getDeclaredConstructor().newInstance();}catch(IllegalAccessException | InvocationTargetException | NoSuchMethodException | InstantiationException e){e.printStackTrace();}return null;};
            robotBaseClass.getMethod("startRobot",Supplier.class).invoke(null,robotConstructor);
        } catch (ClassNotFoundException | IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            e.printStackTrace();
        }
        
        // RobotBase.startRobot(Robot::new);
        
    }
}
