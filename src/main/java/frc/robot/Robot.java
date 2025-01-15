// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
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
     * This method uses reflection to expose private fields of Robot's superclasses.
     * UNDER NO CIRCUMSTANCES SHOULD THIS METHOD BE CALLED IN A COMPETITION! IT IS EXCLUSIVELY FOR
     * DEBUGGING
     * @author Jesse Kane
     */
    private void initTomfoolery(){
        System.out.println("Initializing BS Hacky Workaround (NOTE: SOMETHING HAS GONE SERIOUSLY WRONG IF YOU'RE READING THIS AT A COMPETITION)");
        System.out.println("Exposing IterativeRobotBase private fields");
        for (Field field : IterativeRobotBase.class.getDeclaredFields()){
            System.out.println(field.getName());
            field.setAccessible(true);
            reflectedFields.put(field.getName(),field);
            try {
                System.out.println(field.get(this));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        System.out.println("Exposing IterativeRobotBase Mode enum");
        Class<?> modeClass = IterativeRobotBase.class.getDeclaredClasses()[0];
        for (Object constant : modeClass.getEnumConstants()){
            System.out.println(constant);
            modeConstants.put(constant.toString(),constant);
        }

    }
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     *
     */
    @Override
    public void robotInit() {

        initTomfoolery();
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
    CommandScheduler.getInstance().run();

    //m_robotContainer.updateInterface();
  }

  
  
  //for debugging
  
  private void debugLoopFunc(){
    System.out.println("--------------- BEGIN IRB LOOP FUNC ---------------");
    try {
        System.out.println("Refreshing DS Data");
        DriverStation.refreshData();

        System.out.println("Resetting Watchdog");
        ((Watchdog)reflectedFields.get("m_watchdog").get(this)).reset();

        System.out.println("Resetting DS Control Word");
        ((DSControlWord)reflectedFields.get("m_word").get(this)).refresh();

        // Get current mode
        System.out.println("Getting Current Mode");
        Object mode = modeConstants.get("kNone");
        if (((DSControlWord)reflectedFields.get("m_word").get(this)).isDisabled()) {
            mode = modeConstants.get("kDisabled");
        } else if (((DSControlWord)reflectedFields.get("m_word").get(this)).isAutonomous()) {
            mode = modeConstants.get("kAutonomous");
        } else if (((DSControlWord)reflectedFields.get("m_word").get(this)).isTeleop()) {
            mode = modeConstants.get("kTeleop");
        } else if (((DSControlWord)reflectedFields.get("m_word").get(this)).isTest()) {
            mode = modeConstants.get("kTest");
        }

        if (!((boolean)reflectedFields.get("m_calledDsConnected").get(this)) && ((DSControlWord)reflectedFields.get("m_word").get(this)).isDSAttached()) {
            reflectedFields.get("m_calledDsConnected").set(this,true);
            driverStationConnected();
        }

        // If mode changed, call mode exit and entry functions
        if ((reflectedFields.get("m_lastMode").get(this)) != mode) {
            System.out.println("Detected mode change");
            // Call last mode's exit function
            System.out.println("Exiting old mode");
            switch ((reflectedFields.get("m_lastMode").get(this).toString())) {
                case "kDisabled" -> disabledExit();
                case "kAutonomous" -> autonomousExit();
                case "kTeleop" -> teleopExit();
                case "kTest" -> {
                    if ((boolean)reflectedFields.get("m_lwEnabledInTest").get(this)) {
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
                    ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("disabledInit()");
                }
                case "kAutonomous" -> {
                    autonomousInit();
                    ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("autonomousInit()");
                }
                case "kTeleop" -> {
                    teleopInit();
                    ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("teleopInit()");
                }
                case "kTest" -> {
                    if ((boolean)reflectedFields.get("m_lwEnabledInTest").get(this)) {
                        LiveWindow.setEnabled(true);
                        Shuffleboard.enableActuatorWidgets();
                    }
                    testInit();
                    ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("testInit()");
                }
                default -> {
                    // NOP
                }
            }
            reflectedFields.get("m_lastMode").set(this,mode);
        }
        // Call the appropriate function depending upon the current robot mode
        System.out.println("Calling Mode-Specific periodic function");
        switch (mode.toString()) {
            case "kDisabled" -> {
                DriverStationJNI.observeUserProgramDisabled();
                disabledPeriodic();
                ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("disabledPeriodic()");
            }
            case "kAutonomous" -> {
                DriverStationJNI.observeUserProgramAutonomous();
                autonomousPeriodic();
                ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("autonomousPeriodic()");
            }
            case "kTeleop" -> {
                DriverStationJNI.observeUserProgramTeleop();
                teleopPeriodic();
                ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("teleopPeriodic()");
            }
            case "kTest" -> {
                DriverStationJNI.observeUserProgramTest();
                testPeriodic();
                ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("testPeriodic()");
            }
            default -> {
                // NOP
            }
        }

        System.out.println("Calling robotPeriodic");
        robotPeriodic();
        
        System.out.println("Adding watchdog epochs");
        ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("robotPeriodic()");

        SmartDashboard.updateValues();
        ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("SmartDashboard.updateValues()");
        LiveWindow.updateValues();
        ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("LiveWindow.updateValues()");
        Shuffleboard.update();
        ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("Shuffleboard.update()");

        if (isSimulation()) {
            System.out.println("Calling simulationPeriodic");
            HAL.simPeriodicBefore();
            simulationPeriodic();
            HAL.simPeriodicAfter();
            ((Watchdog)reflectedFields.get("m_watchdog").get(this)).addEpoch("simulationPeriodic()");
        }

        ((Watchdog)reflectedFields.get("m_watchdog").get(this)).disable();

        // Flush NetworkTables
        System.out.println("Flushing networkTables");
        if ((boolean)reflectedFields.get("m_ntFlushEnabled").get(this)) {
            NetworkTableInstance.getDefault().flushLocal();
        }
        
        // Warn on loop time overruns
        if (((Watchdog)reflectedFields.get("m_watchdog").get(this)).isExpired()) {
            ((Watchdog)reflectedFields.get("m_watchdog").get(this)).printEpochs();
        }
    } catch (IllegalAccessException e) {
        e.printStackTrace();
    }
    System.out.println("---------------- END IRB LOOP FUNC ----------------");
  }
  // */

  @Override
  public void loopFunc(){
    //super.loopFunc();
    debugLoopFunc();
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