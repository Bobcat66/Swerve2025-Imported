package org.pihisamurai.lib.debug;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * This singleton class uses reflection to enable full control over the CommandScheduler. Use with caution.
 * 
 * @author Jesse Kane
 */
public final class CommandSchedulerDebugger {

    private final HashMap<String,Field> reflectFields = new HashMap<>();
    private final HashMap<String,Method> reflectMethods = new HashMap<>();
    private Constructor<CommandScheduler> constructor;

    private static CommandSchedulerDebugger instance;

    private CommandSchedulerDebugger(){
        System.out.println("Initializing Command Scheduler Debugger (NOTE: Something has gone seriously wrong if you're reading this at a competition)");
        System.out.println("Exposing CommandScheduler private fields");
        for (Field field : CommandScheduler.class.getDeclaredFields()){
            if (Modifier.isPublic(field.getModifiers())) {continue;} //Skips all public fields
            System.out.println("" + field.getName());
            field.setAccessible(true);
            reflectFields.put("" + field.getName(),field);
        }
        System.out.println("Exposing CommandScheduler private methods");
        for (Method method : CommandScheduler.class.getDeclaredMethods()){
            if (Modifier.isPublic(method.getModifiers())) {continue;} //Skips all public methods
            System.out.println("" + method.getName());
            method.setAccessible(true);
            reflectMethods.put("" + method.getName(),method);
        }
        System.out.println("Exposing CommandScheduler constructor");
        constructor = null;
        try {
            constructor = CommandScheduler.class.getDeclaredConstructor();
            constructor.setAccessible(true);
        } catch (NoSuchMethodException e){
            e.printStackTrace();
        }

    }

    public static CommandSchedulerDebugger getInstance(){
        if (instance == null) {
            instance = new CommandSchedulerDebugger();
        }
        return instance;
    }

    /** Returns a new (non-singleton) instance of the CommandScheduler (USE AT YOUR OWN RISK)*/
    public CommandScheduler getNewCSInstance() {
        try {
            return (CommandScheduler) constructor.newInstance();
        } catch (IllegalAccessException | InstantiationException | InvocationTargetException e) {
            e.printStackTrace();
            return null;
        }
        
    }

    /** Returns a monitor of the specified field */
    public <T> FieldMonitor<T> getMonitor(String fieldName){
        return new FieldMonitor<T>(reflectFields.get(fieldName),CommandScheduler.getInstance());
    }

    
}
