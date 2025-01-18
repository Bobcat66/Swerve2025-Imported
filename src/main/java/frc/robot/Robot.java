// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import debug.DebugInstrumentator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private HashMap<String,Field> reflectedFields = new HashMap<>();
    private HashMap<String,Method> reflectedMethods = new HashMap<>();
    private HashMap<String,Object> modeConstants = new HashMap<>();

    /** 
     * Your so-called "access modifiers" can't stop me if I can't read
     * @author Jesse Kane
     */
    private void initTomfoolery(){
        System.out.println("Initializing Tomfoolery (NOTE: THIS IS FOR DEBUGGING ONLY, SOMETHING HAS GONE SERIOUSLY WRONG IF YOU'RE READING THIS AT A COMPETITION)");
        System.out.println("Exposing IterativeRobotBase private fields");
        for (Field field : IterativeRobotBase.class.getDeclaredFields()){
            if (Modifier.isPublic(field.getModifiers())) {continue;} //Skips all public fields
            System.out.println("IterativeRobotBase$" + field.getName());
            field.setAccessible(true);
            reflectedFields.put("IterativeRobotBase$" + field.getName(),field);
        }
        System.out.println("Exposing IterativeRobotBase Mode enum");
        Class<?> modeClass = IterativeRobotBase.class.getDeclaredClasses()[0];
        for (Object constant : modeClass.getEnumConstants()){
            System.out.println(constant);
            modeConstants.put(constant.toString(),constant);
        }
        System.out.println("Exposing CommandScheduler private fields");
        for (Field field : CommandScheduler.class.getDeclaredFields()){
            if (Modifier.isPublic(field.getModifiers())) {continue;} //Skips all public fields
            System.out.println("CommandScheduler$" + field.getName());
            field.setAccessible(true);
            reflectedFields.put("CommandScheduler$" + field.getName(),field);
        }
        System.out.println("Exposing CommandScheduler private methods");
        for (Method method : CommandScheduler.class.getDeclaredMethods()){
            if (Modifier.isPublic(method.getModifiers())) {continue;} //Skips all public methods
            System.out.println("CommandScheduler$" + method.getName());
            method.setAccessible(true);
            reflectedMethods.put("CommandScheduler$" + method.getName(),method);
        }
        System.out.println("Exposing FollowPathCommand private fields");
        for (Field field : FollowPathCommand.class.getDeclaredFields()){
            if (Modifier.isPublic(field.getModifiers())) {continue;} //Skips all public fields
            System.out.println("FollowPathCommand$" + field.getName());
            field.setAccessible(true);
            reflectedFields.put("FollowPathCommand$" + field.getName(),field);
        }

    }
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     *
     */
    @Override
    public void robotInit() {

        //initTomfoolery();
        // Configure AdvantageKit. This must be done BEFORE any other instatiation
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

    // Set up data receivers & replay source
    switch (Constants.Akit.currentMode) {
        case 0:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case 1:
            // Running a physics simulator, log to NT
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case 2:
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
    }

    // Start AdvantageKit logger
    Logger.start();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

   /** 
    * 
    * A debugging version of CommandScheduler's run command, enables detailed logging of the CommandScheduler, to help with diagnosing loop overruns
    * 
    * @author Jesse Kane
    */
    @SuppressWarnings("unchecked")
    private void debugCSRun(){
        System.out.println("--------------- BEGIN COMMANDSCHEDULER RUN ---------------");
        try {
            if ((boolean)reflectedFields.get("CommandScheduler$m_disabled").get(CommandScheduler.getInstance())) {
                return;
            }
            ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).reset();

            // Run the periodic method of all registered subsystems.
            System.out.println("Running subsystem periodic methods");
            for (Subsystem subsystem : ((Map<Subsystem, Command>)reflectedFields.get("CommandScheduler$m_subsystems").get(CommandScheduler.getInstance())).keySet()) {
                System.out.println("Running periodic method of subsystem " + subsystem);
                subsystem.periodic();
                if (RobotBase.isSimulation()) {
                    System.out.println("Running sim periodic method of subsystem " + subsystem);
                    subsystem.simulationPeriodic();
                }
                ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).addEpoch(subsystem.getName() + ".periodic()");
            }

            // Cache the active instance to avoid concurrency problems if setActiveLoop() is called from
            // inside the button bindings.
            EventLoop loopCache = ((EventLoop)reflectedFields.get("CommandScheduler$m_activeButtonLoop").get(CommandScheduler.getInstance()));
            // Poll buttons for new commands to add.
            loopCache.poll();
            ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).addEpoch("buttons.run()");

            reflectedFields.get("CommandScheduler$m_inRunLoop").set(CommandScheduler.getInstance(),true);
            boolean isDisabled = RobotState.isDisabled();
            // Run scheduled commands, remove finished commands.
            for (Iterator<Command> iterator = ((Set<Command>)reflectedFields.get("CommandScheduler$m_scheduledCommands").get(CommandScheduler.getInstance())).iterator(); iterator.hasNext(); ) {
                Command command = iterator.next();

                if (isDisabled && !command.runsWhenDisabled()) {
                    System.out.println(reflectedMethods.get("CommandScheduler$cancel").getParameterCount());
                    reflectedMethods.get("CommandScheduler$cancel").invoke(CommandScheduler.getInstance(),command,((Optional<Command>)reflectedFields.get("CommandScheduler$kNoInterruptor").get(CommandScheduler.getInstance())));
                    continue;
                }


                System.out.println("Executing Command " + command);
                command.execute();
                for (Consumer<Command> action : (List<Consumer<Command>>)reflectedFields.get("CommandScheduler$m_executeActions").get(CommandScheduler.getInstance())) {
                    action.accept(command);
                }
                ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).addEpoch(command.getName() + ".execute()");
                if (command.isFinished()) {
                    ((Set<Command>)reflectedFields.get("CommandScheduler$m_endingCommands").get(CommandScheduler.getInstance())).add(command);
                    command.end(false);
                    for (Consumer<Command> action : ((List<Consumer<Command>>)reflectedFields.get("CommandScheduler$m_finishActions").get(CommandScheduler.getInstance()))) {
                        action.accept(command);
                    }
                    ((Set<Command>)reflectedFields.get("CommandScheduler$m_endingCommands").get(CommandScheduler.getInstance())).remove(command);
                    iterator.remove();

                    ((Map<Subsystem,Command>)reflectedFields.get("CommandScheduler$m_requirements").get(CommandScheduler.getInstance())).keySet().removeAll(command.getRequirements());
                    ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).addEpoch(command.getName() + ".end(false)");
                }
            }
            reflectedFields.get("CommandScheduler$m_inRunLoop").set(CommandScheduler.getInstance(),false);

            // Schedule/cancel commands from queues populated during loop
            for (Command command : (Set<Command>)reflectedFields.get("CommandScheduler$m_toSchedule").get(CommandScheduler.getInstance())) {
                System.out.println("Scheduling queued command " + command);
                CommandScheduler.getInstance().schedule(command);
            }

            for (int i = 0; i < ((List<Command>)reflectedFields.get("CommandScheduler$m_toCancelCommands").get(CommandScheduler.getInstance())).size(); i++) {
                System.out.println("Cancelling command " + ((List<Command>)reflectedFields.get("CommandScheduler$m_toCancelCommands").get(CommandScheduler.getInstance())).get(i));
                reflectedMethods.get("CommandScheduler$cancel").invoke(
                    CommandScheduler.getInstance(),
                    ((List<Command>)reflectedFields.get("CommandScheduler$m_toCancelCommands").get(CommandScheduler.getInstance())).get(i),
                    ((List<Optional<Command>>)reflectedFields.get("CommandScheduler$m_toCancelInterruptors").get(CommandScheduler.getInstance())).get(i)
                );
            }

            ((Set<Command>)reflectedFields.get("CommandScheduler$m_toSchedule").get(CommandScheduler.getInstance())).clear();
            ((List<Command>)reflectedFields.get("CommandScheduler$m_toCancelCommands").get(CommandScheduler.getInstance())).clear();
            ((List<Optional<Command>>)reflectedFields.get("CommandScheduler$m_toCancelInterruptors").get(CommandScheduler.getInstance())).clear();

            // Add default commands for un-required registered subsystems.
            for (Map.Entry<Subsystem, Command> subsystemCommand : ((Map<Subsystem, Command>)reflectedFields.get("CommandScheduler$m_subsystems").get(CommandScheduler.getInstance())).entrySet()) {
                if (!((Map<Subsystem,Command>)reflectedFields.get("CommandScheduler$m_requirements").get(CommandScheduler.getInstance())).containsKey(subsystemCommand.getKey())
                    && subsystemCommand.getValue() != null) {
                    System.out.println("Scheduling subsystem " + subsystemCommand.getKey() + " default command " + subsystemCommand.getValue());
                    CommandScheduler.getInstance().schedule(subsystemCommand.getValue());
                }
            }

            ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).disable();
            if (((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).isExpired()) {
                System.out.println("CommandScheduler loop overrun");
                ((Watchdog)reflectedFields.get("CommandScheduler$m_watchdog").get(CommandScheduler.getInstance())).printEpochs();
            }
    } catch (IllegalAccessException | InvocationTargetException e) {
        e.printStackTrace();
    }
    System.out.println("---------------- END COMMANDSCHEDULER RUN ----------------");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    try{
        Class<?> clazz = DebugInstrumentator.getInstance().classLoader.loadClass("edu.wpi.first.wpilibj2.command.CommandScheduler");
        Object inst = clazz.getDeclaredMethod("getInstance").invoke(null);
        clazz.getDeclaredMethod("run").invoke(inst);

    } catch (Exception e) {
        e.printStackTrace();
    }
    
    //CommandScheduler.getInstance().run();
    //debugCSRun();

    //m_robotContainer.updateInterface();
  }

  
  
  //for debugging
  
  private void debugLoopFunc(){
    System.out.println("--------------- BEGIN IRB LOOP FUNC ---------------");
    try {
        System.out.println("Refreshing DS Data");
        DriverStation.refreshData();

        System.out.println("Resetting Watchdog");
        ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).reset();

        System.out.println("Resetting DS Control Word");
        ((DSControlWord)reflectedFields.get("IterativeRobotBase$m_word").get(this)).refresh();

        // Get current mode
        System.out.println("Getting Current Mode");
        Object mode = modeConstants.get("kNone");
        if (((DSControlWord)reflectedFields.get("IterativeRobotBase$m_word").get(this)).isDisabled()) {
            mode = modeConstants.get("kDisabled");
        } else if (((DSControlWord)reflectedFields.get("IterativeRobotBase$m_word").get(this)).isAutonomous()) {
            mode = modeConstants.get("kAutonomous");
        } else if (((DSControlWord)reflectedFields.get("IterativeRobotBase$m_word").get(this)).isTeleop()) {
            mode = modeConstants.get("kTeleop");
        } else if (((DSControlWord)reflectedFields.get("IterativeRobotBase$m_word").get(this)).isTest()) {
            mode = modeConstants.get("kTest");
        }

        if (!((boolean)reflectedFields.get("IterativeRobotBase$m_calledDsConnected").get(this)) && ((DSControlWord)reflectedFields.get("IterativeRobotBase$m_word").get(this)).isDSAttached()) {
            reflectedFields.get("IterativeRobotBase$m_calledDsConnected").set(this,true);
            driverStationConnected();
        }

        // If mode changed, call mode exit and entry functions
        if ((reflectedFields.get("IterativeRobotBase$m_lastMode").get(this)) != mode) {
            System.out.println("Detected mode change");
            // Call last mode's exit function
            System.out.println("Exiting old mode");
            switch ((reflectedFields.get("IterativeRobotBase$m_lastMode").get(this).toString())) {
                case "kDisabled" -> disabledExit();
                case "kAutonomous" -> autonomousExit();
                case "kTeleop" -> teleopExit();
                case "kTest" -> {
                    if ((boolean)reflectedFields.get("IterativeRobotBase$m_lwEnabledInTest").get(this)) {
                        LiveWindow.setEnabled(false);
                        Shuffleboard.disableActuatorWidgets();
                    }
                    testExit();
                }
                default -> {
                    // NOP
                }
            }

            // Call current mode's entry function
            System.out.println("Initializing new mode");
            switch (mode.toString()) {
                case "kDisabled" -> {
                    disabledInit();
                    ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("disabledInit()");
                }
                case "kAutonomous" -> {
                    autonomousInit();
                    ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("autonomousInit()");
                }
                case "kTeleop" -> {
                    teleopInit();
                    ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("teleopInit()");
                }
                case "kTest" -> {
                    if ((boolean)reflectedFields.get("IterativeRobotBase$m_lwEnabledInTest").get(this)) {
                        LiveWindow.setEnabled(true);
                        Shuffleboard.enableActuatorWidgets();
                    }
                    testInit();
                    ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("testInit()");
                }
                default -> {
                    // NOP
                }
            }
            reflectedFields.get("IterativeRobotBase$m_lastMode").set(this,mode);
        }
        // Call the appropriate function depending upon the current robot mode
        System.out.println("Calling Mode-Specific periodic function");
        switch (mode.toString()) {
            case "kDisabled" -> {
                DriverStationJNI.observeUserProgramDisabled();
                disabledPeriodic();
                ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("disabledPeriodic()");
            }
            case "kAutonomous" -> {
                DriverStationJNI.observeUserProgramAutonomous();
                autonomousPeriodic();
                ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("autonomousPeriodic()");
            }
            case "kTeleop" -> {
                DriverStationJNI.observeUserProgramTeleop();
                teleopPeriodic();
                ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("teleopPeriodic()");
            }
            case "kTest" -> {
                DriverStationJNI.observeUserProgramTest();
                testPeriodic();
                ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("testPeriodic()");
            }
            default -> {
                // NOP
            }
        }

        System.out.println("Calling robotPeriodic");
        robotPeriodic();
        
        System.out.println("Adding watchdog epochs");
        ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("robotPeriodic()");

        SmartDashboard.updateValues();
        ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("SmartDashboard.updateValues()");
        LiveWindow.updateValues();
        ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("LiveWindow.updateValues()");
        Shuffleboard.update();
        ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("Shuffleboard.update()");

        if (isSimulation()) {
            System.out.println("Calling simulationPeriodic");
            HAL.simPeriodicBefore();
            simulationPeriodic();
            HAL.simPeriodicAfter();
            ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).addEpoch("simulationPeriodic()");
        }

        ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).disable();

        // Flush NetworkTables
        System.out.println("Flushing networkTables");
        if ((boolean)reflectedFields.get("IterativeRobotBase$m_ntFlushEnabled").get(this)) {
            NetworkTableInstance.getDefault().flushLocal();
        }
        
        // Warn on loop time overruns
        if (((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).isExpired()) {
            ((Watchdog)reflectedFields.get("IterativeRobotBase$m_watchdog").get(this)).printEpochs();
        }
    } catch (IllegalAccessException e) {
        e.printStackTrace();
    }
    System.out.println("---------------- END IRB LOOP FUNC ----------------");
  }
  // */

  @Override
  public void loopFunc(){
    super.loopFunc();
    //debugLoopFunc();
  }
  
  

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic(){}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}