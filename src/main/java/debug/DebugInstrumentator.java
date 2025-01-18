package debug;

import javassist.CannotCompileException;
import javassist.ClassClassPath;
import javassist.ClassPool;
import javassist.CtClass;
import javassist.CtMethod;
import javassist.NotFoundException;

import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.lang.management.RuntimeMXBean;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Dynamically adds instrumentation code to the WPILIB bytecode. Instrumentation methods should ALWAYS be called in the robot's main method, prior to any other initialization. This is necessary to ensure that
 * the JVM has not loaded the CommandScheduler class yet. WIP
 * 
 * @author Jesse Kane
 */
public final class DebugInstrumentator {
    private static DebugInstrumentator instance = null;
    private ClassPool pool;
    private CtClass csClass;
    public DebugClassLoader classLoader;

    private DebugInstrumentator() {
        System.out.println("Initializing CommandScheduler Instrumentator");
        System.out.println("Enabling Debugging Classloader");
        classLoader = new DebugClassLoader();
        Thread.currentThread().setContextClassLoader(classLoader);
        System.out.println("Fetching CommandScheduler class object");
        try{
            pool = ClassPool.getDefault();
            pool.appendClassPath(new ClassClassPath(CommandScheduler.class));
            csClass = ClassPool.getDefault().get("edu.wpi.first.wpilibj2.command.CommandScheduler");
        } catch (NotFoundException e) {
            e.printStackTrace();
        }
        RuntimeMXBean runtimeMxBean = ManagementFactory.getRuntimeMXBean();
        Map<String,String> properties = runtimeMxBean.getSystemProperties();

        System.out.println("System Properties:");
        for (String key : properties.keySet()) {
            System.out.println(key + " : " + properties.get(key));
        }
        List<String> args = runtimeMxBean.getInputArguments();
        System.out.println("Input Arguments:");
        for (String arg : args){
            System.out.println(arg);
        }
    }

    public static DebugInstrumentator getInstance(){
        if (instance == null) {
            instance = new DebugInstrumentator();
        }
        return instance;
    }

    /** Dynamically modifies JVM bytecode to add debugging telemetry to CommandScheduler's run method.*/
    public void CSrunInstrumentation() {
        String methodBody = "{"
        + "System.out.println(\"------------ BEGIN COMMANDSCHEDULER RUN ------------\");"
        + "if (m_disabled) {"
        + "    return;"
        + "}"
        + "System.out.println(\"Resetting watchdog\");"
        + "m_watchdog.reset();"
        + "for (Subsystem subsystem : m_subsystems.keySet()) {"
        + "    System.out.println(\"Running periodic of \" + subsystem.toString());"
        + "    subsystem.periodic();"
        + "    if (RobotBase.isSimulation()) {"
        + "        System.out.println(\"Running sim periodic of \" + subsystem.toString());"
        + "        subsystem.simulationPeriodic();"
        + "    }"
        + "    m_watchdog.addEpoch(subsystem.getName() + \".periodic()\");"
        + "}"
        + "System.out.println(\"Polling Buttons\");"
        + "EventLoop loopCache = m_activeButtonLoop;"
        + "loopCache.poll();"
        + "m_watchdog.addEpoch(\"buttons.run()\");"
        + "m_inRunLoop = true;"
        + "boolean isDisabled = RobotState.isDisabled();"
        + "Iterator<Command> iterator = m_scheduledCommands.iterator();"
        + "while (iterator.hasNext(); ) {"
        + "    Command command = iterator.next();"
        + "    if (isDisabled && !command.runsWhenDisabled()) {"
        + "        cancel(command, kNoInterruptor);"
        + "        continue;"
        + "    }"
        + "    System.out.println(\"Executing command \" + command.toString());"
        + "    command.execute();"
        + "    for (Consumer<Command> action : m_executeActions) {"
        + "        action.accept(command);"
        + "    }"
        + "    m_watchdog.addEpoch(command.getName() + \".execute()\");"
        + "    if (command.isFinished()) {"
        + "        m_endingCommands.add(command);"
        + "        command.end(false);"
        + "        for (Consumer<Command> action : m_finishActions) {"
        + "            action.accept(command);"
        + "        }"
        + "        m_endingCommands.remove(command);"
        + "        iterator.remove();"
        + "        m_requirements.keySet().removeAll(command.getRequirements());"
        + "        m_watchdog.addEpoch(command.getName() + \".end(false)\");"
        + "    }"
        + "}"
        + "m_inRunLoop = false;"
        + "for (Command command : m_toSchedule) {"
        + "    schedule(command);"
        + "}"
        + "for (int i = 0; i < m_toCancelCommands.size(); i++) {"
        + "    cancel(m_toCancelCommands.get(i), m_toCancelInterruptors.get(i));"
        + "}"
        + "m_toSchedule.clear();"
        + "m_toCancelCommands.clear();"
        + "m_toCancelInterruptors.clear();"
        + "for (Map.Entry<Subsystem, Command> subsystemCommand : m_subsystems.entrySet()) {"
        + "    if (!m_requirements.containsKey(subsystemCommand.getKey()) && subsystemCommand.getValue() != null) {"
        + "        schedule(subsystemCommand.getValue());"
        + "    }"
        + "}"
        + "System.out.println(\"Disabling watchdog\");"
        + "m_watchdog.disable();"
        + "if (m_watchdog.isExpired()) {"
        + "    System.out.println(\"CommandScheduler loop overrun\");"
        + "    m_watchdog.printEpochs();"
        + "}"
        + "System.out.println(\"------------- END COMMANDSCHEDULER RUN -------------\");"
        + "}";
        try{
            csClass.getDeclaredMethod("run").setBody(methodBody);
            
        } catch (NotFoundException | CannotCompileException e) {
            e.printStackTrace();
            System.out.println(methodBody);
        }
    }

    public void testInstrumentation(){
        try{
            csClass.getDeclaredMethod("run").insertAt(266,"System.out.println(\"Running Subsystem periodic methods\");");
            csClass.getDeclaredMethod("run").insertAt(266,"System.out.println(\"------------ BEGIN COMMANDSCHEDULER RUN ------------\");");
            csClass.getDeclaredMethod("run").insertAt(273,"System.out.println(\"Running periodic method of subsystem \");");
            System.out.println(csClass.getDeclaredMethod("run").getMethodInfo().getCodeAttribute().toString());
        } catch (NotFoundException | CannotCompileException e) {
            e.printStackTrace();
        }
    }

    /** Dynamically modifies JVM bytecode to add debugging telemetry to CommandScheduler's schedule method.*/
    public void registerSchedulingTelemetry(){
        try {
            csClass.getDeclaredMethod("schedule",new CtClass[] {ClassPool.getDefault().get("edu.wpi.first.wpilibj2.command.Command")}).setBody("""
            {
                System.out.println("CommandScheduler: Attempting to schedule command" + command.toString());
                if (command == null) {
                    DriverStation.reportWarning("Tried to schedule a null command", true);
                    return;
                }
                if (m_inRunLoop) {
                    m_toSchedule.add(command);
                    return;
                }

                requireNotComposed(command);

                // Do nothing if the scheduler is disabled, the robot is disabled and the command doesn't
                // run when disabled, or the command is already scheduled.
                if (m_disabled || isScheduled(command) || RobotState.isDisabled() && !command.runsWhenDisabled()) {
                    System.out.println("CommandScheduler: failed to schedule command " + command.toString() + "because");
                    if (m_disabled) System.out.println("CommandScheduler is disabled");
                    if (isScheduled(command)) System.out.println("Command is already scheduled");
                    if (RobotState.isDisabled && !command.runsWhenDisabled()) System.out.println("Robot is disabled and Command does not run while disabled");
                    return;
                }

                Set<Subsystem> requirements = command.getRequirements();

                // Schedule the command if the requirements are not currently in-use.
                if (Collections.disjoint(m_requirements.keySet(), requirements)) {
                    initCommand(command, requirements);
                    System.out.println("CommandScheduler: successfully scheduled command " + command.toString());
                } else {
                    // Else check if the requirements that are in use have all have interruptible commands,
                    // and if so, interrupt those commands and schedule the new command.
                    for (Subsystem requirement : requirements) {
                        Command requiring = requiring(requirement);
                        if (requiring != null && requiring.getInterruptionBehavior() == InterruptionBehavior.kCancelIncoming) {
                            System.out.println("CommandScheduler: failed to schedule command " + command.toString() + "because");
                            System.out.println("Required subsystem " + requirement.toString() + " already in use by command " + requiring.toString());
                            return;
                        }
                    }
                    for (Subsystem requirement : requirements) {
                        Command requiring = requiring(requirement);
                        if (requiring != null) {
                            cancel(requiring, Optional.of(command));
                        }
                    }
                    initCommand(command, requirements);
                    System.out.println("CommandScheduler: successfully scheduled command " + command.toString());
                }
            }""");
        } catch (NotFoundException | CannotCompileException e) {
            e.printStackTrace();
        }
    }

    /** Loads the CommandScheduler's bytecode. THE CS INSTRUMENTATOR CAN NO LONGER BE USED AFTER THIS METHOD IS INVOKED */
    public void load(){
        try {
            pool.toClass(csClass);
            //classLoader.toClass(csClass.getName(),csClass.toBytecode(),0,csClass.toBytecode().length,csClass.getClass().getProtectionDomain());
        } catch (CannotCompileException e) {
            e.printStackTrace();
        }
    }
}
