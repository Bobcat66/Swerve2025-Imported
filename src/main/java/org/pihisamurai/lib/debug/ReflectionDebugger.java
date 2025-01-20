package org.pihisamurai.lib.debug;

import java.lang.reflect.Field;
import java.lang.reflect.InaccessibleObjectException;
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

    public ClassMonitor getClassMonitor(Class<?> clazz) {
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
        LOGGER.info("Attaching to " + clazz.getCanonicalName());
        StringBuilder logString = new StringBuilder();
        logString.append("Attached to " + clazz.getCanonicalName() + "\n");
        attachedClasses.put(clazz,new ClassMonitor(clazz));
        
        logString.append("Exposed Fields:\n");
        for (Field field : clazz.getDeclaredFields()){
            try {
                field.setAccessible(true);
            } catch (InaccessibleObjectException e) {
                LOGGER.warning("Field " + clazz.getCanonicalName() + "$" + field.getName() + " is inaccessible and will not be monitored by the Debugger.");
                continue;
            }
            logString.append(field.getName() + "\n");
            attachedClasses.get(clazz).fields.put(field.getName(),field);
        }
        logString.append("Exposed Methods:\n");
        for (Method method : clazz.getDeclaredMethods()){
            try {
                method.setAccessible(true);
            } catch (InaccessibleObjectException e) {
                LOGGER.warning("Method " + clazz.getCanonicalName() + "$" + method.getName() + " is inaccessible and will not be monitored by the Debugger.");
                continue;
            }
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
            attachedClasses.get(clazz).methods.put(tsb.toString(),method);
            logString.append(tsb.toString() + "\n");
        }
        LOGGER.info(logString.toString());
        return attachedClasses.get(clazz);
    }
}
