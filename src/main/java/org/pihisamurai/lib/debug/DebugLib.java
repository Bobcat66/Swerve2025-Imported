package org.pihisamurai.lib.debug;

import java.lang.reflect.Field;
import java.lang.reflect.InaccessibleObjectException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.logging.Logger;

import org.apache.commons.lang3.NotImplementedException;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.pihisamurai.lib.debug.DebugLib.ReflectionDebugger.ClassMonitor;

/** This is a single-file implementation of the entire Debugging Library, designed to be as portable as possible. To install the library in a project, simply drop this file in and change the package header
 * 
 * @author Jesse Kane
 */
public final class DebugLib {

    /**
     * A singleton class that directly manages reflection
     */
    public static final class ReflectionDebugger {

        public static class ClassMonitor {

            private final Map<String,Field> fields = new HashMap<>();
            private final Map<String,Method> methods = new HashMap<>();
            private final Class<?> clazz;

            private ClassMonitor(Class<?> clazz) {
                this.clazz = clazz;
            }

            public <U> FieldMonitor<U> getFieldMonitor(String fieldName,Object instance) {
                return new FieldMonitor<U>(fields.get(fieldName),instance);
            }
        
            @SuppressWarnings("unchecked")
            public <U> ObjectMonitor<U> getObjectMonitorFromField(String fieldName,Object instance) {
                try {
                    return ObjectMonitor.of((U)fields.get(fieldName).get(instance));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                    return null;
                }
            }
        
            public <U> ReflectedClosure<U> getClosure(String methodName,Object instance) {
                Method method = methods.get(methodName);
                return new ReflectedClosure<U>(instance, method, method.getParameterTypes());
            }
        
            public <U> ReflectedClosure<U> getStaticClosure(String methodName) {
                Method method = methods.get(methodName);
                return new ReflectedClosure<U>(null,method, method.getParameterTypes());
            }
        }
        
        private final Map<Class<?>,ClassMonitor> attachedClasses = new HashMap<>();

        private static ReflectionDebugger instance;
    
        private final Logger LOGGER;

        private ReflectionDebugger() {
            LOGGER = Logger.getLogger("ReflectionDebugger");
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

    /**
     * Contains utility methods for debugging PathPlanner
     */
    public final class PPDebugging {
        public static PathPlannerTrajectory getTrajectoryFromFPC(FollowPathCommand fpc) {
            return ReflectionDebugger.getInstance().getClassMonitor(FollowPathCommand.class).<PathPlannerTrajectory>getObjectMonitorFromField("trajectory",fpc).get();
        }

        public static Command getAutoCommand(PathPlannerAuto command) {
            return ReflectionDebugger.getInstance().getClassMonitor(PathPlannerAuto.class).<Command>getObjectMonitorFromField("autoCommand",command).get();
        }

        public static Timer getTimerFromFPC(FollowPathCommand fpc){
            return ReflectionDebugger.getInstance().getClassMonitor(FollowPathCommand.class).<Timer>getObjectMonitorFromField("timer",fpc).get();
        }

        /** Returns the Nth command from a sequential command group. WARNING: The sequential command group should NOT be scheduled after calling this method on it */
        public static Optional<Command> getNthCommandFromSCG(SequentialCommandGroup commandGroup, int n){
            List<Command> commandList = ReflectionDebugger.getInstance().getClassMonitor(SequentialCommandGroup.class).<List<Command>>getObjectMonitorFromField("m_commands",commandGroup).get();
            try {
                CommandScheduler.getInstance().clearComposedCommands();
                return Optional.of(commandList.get(n));
            } catch (IndexOutOfBoundsException e) {
                return Optional.empty();
            }
        }

        public static List<PathPlannerTrajectoryState> getPPTStates(PathPlannerTrajectory trajectory) {
            return ReflectionDebugger.getInstance().getClassMonitor(PathPlannerTrajectory.class).<List<PathPlannerTrajectoryState>>getObjectMonitorFromField("states",trajectory).get();
        }

        private PPDebugging(){ 
            throw new NotImplementedException("This is a utility class!");
        }
    }

    public static class ObjectMonitor<T> {

        private final T object;
        private final Class<?> clazz;
        private final ClassMonitor clazzMonitor;
        private ObjectMonitor(T object) {
            this.object = object;
            this.clazz = object.getClass();
            this.clazzMonitor = ReflectionDebugger.getInstance().getClassMonitor(this.clazz);
        }
        /** Wraps an object inside an objectMonitor */
        public static <U> ObjectMonitor<U> of(U object){
            return new ObjectMonitor<>(object);
        }

        /**returns the object*/
        public T get() {
            return object;
        }
    
        /** Flat Map */
        public <R> ObjectMonitor<R> fmap(Function<T,ObjectMonitor<R>> mapper) {
            return mapper.apply(object);
        }

        /** Regular Map */
        public <R> ObjectMonitor<R> map(Function<T,R> mapper) {
            return ObjectMonitor.of(mapper.apply(object));
        }

        /** Gets field from the monitored object */
        public <U> ObjectMonitor<U> getField(String fieldName){
            return clazzMonitor.getObjectMonitorFromField(fieldName, object);
        }

        /** Gets closure from a method of the monitored object*/
        public <U> ReflectedClosure<U> getClosure(String methodName){
            return clazzMonitor.getClosure(methodName, object);
        }

        /** Gets closure from a static method of the monitored object */
        public <U> ReflectedClosure<U> getStaticClosure(String methodName){
            return clazzMonitor.getClosure(methodName,null);
        }
    }

    /**
    * A reference to a field within an object. This is meant to make it easier to monitor the state of a private field.
    * 
    * The term "value" refers to the object contained within the monitored field
    * 
    * WARNING: ABSOLUTELY NO GUARANTEES ARE PROVIDED REGARDING MEMORY OR NULL-POINTER SAFETY. USE AT YOUR OWN RISK
    * @param <T> the type of the field being referenced
    */
    @SuppressWarnings("unchecked")
    public static class FieldMonitor<T> {

        private final Field field; //The field being monitored
        private final Object instance; //The object that owns the monitored field

        public FieldMonitor(Field field, Object instance){
            field.setAccessible(true);
            this.field = field;
            this.instance = instance;
        }

        /** returns the value of the monitored field, wrapped in an objectMonitor*/
        public ObjectMonitor<T> get() {
            try{
                return ObjectMonitor.of((T)field.get(instance));
            } catch (IllegalAccessException e){
                e.printStackTrace();
                return null;
            }
        }

        /** 
         * Sets the value of the monitored field
         * 
         * @return a boolean indicating the success of the operation
         */
        public boolean set(T newVal) {
            try {
                field.set(instance,newVal);
                return true;
            } catch (IllegalAccessException e) {
                e.printStackTrace();
                return false;
            }
        } 
    }

    public static class ReflectedClosure<T> {
        
        private final Class<?>[] paramTypes;
        private final Method boundMethod;
        private final Object boundEnv;

        public ReflectedClosure(Object env, Method method, Class<?>... paramTypes){
            this.paramTypes = paramTypes;
            this.boundMethod = method;
            this.boundEnv = env;
        }

        @SuppressWarnings("unchecked")
        public T invoke(Object... params) {

            //Checks parameters for correctness
            if (params.length != paramTypes.length) {
                throw new IllegalArgumentException("Invalid Number of Arguments");
            }
            for (int i = 0 ; i < params.length ; i++) {
                try {
                    paramTypes[i].cast(params[i]);
                } catch (ClassCastException e){
                    throw e;
                }
            }

            try {
                return (T)boundMethod.invoke(boundEnv,params);
            } catch (InvocationTargetException | IllegalAccessException e) {
                e.printStackTrace();
                return null;
            }
        }
    }



}
