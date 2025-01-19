package org.pihisamurai.lib.debug;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Parameter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.logging.Logger;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A singleton class designed to use reflection to access private methods and fields
 * 
 * @author Jesse Kane
 */
public final class ReflectionDebugger {

    public static class ClassMonitor {
        public Map<String,Field> fields = new HashMap<>();
        public Map<String,Method> methods = new HashMap<>();
        public <T> FieldMonitor<T> getMonitor(String fieldName,Object instance) {
            return new FieldMonitor<T>(fields.get(fieldName),instance);
        }
    }

    private final Map<Class<?>,ClassMonitor> attachedClasses = new HashMap<>();

    private static ReflectionDebugger instance;
    
    private final Logger LOGGER;

    private ReflectionDebugger() {
        LOGGER = Logger.getLogger("ReflectionDebugger");
        attach(PathPlannerAuto.class);
        attach(FollowPathCommand.class);
        attach(SequentialCommandGroup.class);
    }

    public static ReflectionDebugger getInstance() {
        if (instance == null) {
            instance = new ReflectionDebugger();
        }
        return instance;
    }

    public ClassMonitor getMonitor(Class<?> clazz) {
        if (attachedClasses.keySet().contains(clazz)) {
            return attachedClasses.get(clazz);
        } else {
            return attach(clazz);
        }
    }

    private ClassMonitor attach(Class<?> clazz){
        //LOGGER.info("Attaching to " + clazz.getSimpleName());
        if (attachedClasses.keySet().contains(clazz)) {
            LOGGER.warning("Debugger is already attached to " + clazz.getSimpleName());
            return null;
        }
        StringBuilder logString = new StringBuilder();
        logString.append("Attaching to " + clazz.getCanonicalName() + "\n");
        attachedClasses.put(clazz,new ClassMonitor());
        
        logString.append("Exposing Fields:\n");
        for (Field field : clazz.getDeclaredFields()){
            logString.append(field.getName() + "\n");
            field.setAccessible(true);
            attachedClasses.get(clazz).fields.put(field.getName(),field);
        }
        logString.append("Exposing Methods:\n");
        for (Method method : clazz.getDeclaredMethods()){
            StringBuilder tsb = new StringBuilder(method.getName() + "{");
            boolean params = false;
            for (Parameter param : method.getParameters()){
                params = true;
                tsb.append(param.getType().getSimpleName() + ",");
            }
            //Removes trailing comma
            if (params) {
                tsb.deleteCharAt(tsb.length()-1);
            }
            tsb.append("}");
            method.setAccessible(true);
            attachedClasses.get(clazz).methods.put(tsb.toString(),method);
            logString.append(tsb.toString() + "\n");
        }
        LOGGER.info(logString.toString());
        return attachedClasses.get(clazz);
    }
}
