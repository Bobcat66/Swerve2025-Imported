// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive.WheelRadius;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OIConstants {
        public static class Driver {
            public static final int kDriverControllerPort = 0;
        }
    }

    public static class Akit {
        public static final int currentMode = 0;
    }

    public static class DriveConstants {
        public static final int odometryFrequencyHz = 250;
        public static final double wheelBase = Units.inchesToMeters(27.5); //Meters
        public static final double trackWidth = Units.inchesToMeters(19.5); //Meters
        //public static final double wheelRadius = 0.0508; //Meters
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        public static class GyroConstants {
            public static final int kGyroPort = 9;
        }

        public static class AutoConstants {
            public static final RobotConfig ppConfig = new RobotConfig(
                100.0,
                1.0,
                new ModuleConfig(
                    ModuleConstants.Common.Drive.WheelRadius,
                    ModuleConstants.Common.Drive.MaxModuleSpeed, 
                    ModuleConstants.Common.Drive.WheelCOF,
                    DCMotor.getNEO(1),
                    ModuleConstants.Common.Drive.CurrentLimit,
                    1
                ),
                moduleTranslations
            );
            public static class PIDControl {
                public static class Trans {
                    public static final double kP = 5.0;
                    public static final double kI = 0.0;
                    public static final double kD = 0.0;
                }
                public static class Rot {
                    public static final double kP = 5.0;
                    public static final double kI = 0.0;
                    public static final double kD = 0.0;
                }
            }
        }

        public static class ModuleConstants {
            public static class Common {
                public static class Drive {
                    public static final int CurrentLimit = 60;
                    public static final double gearRatio = 6.75;
                    public static final double VoltageCompensation = 12;
                    public static final double MaxModuleSpeed = 14.0; //Maximum attainable module speed
                    public static final double WheelRadius = Units.inchesToMeters(4); //Meters
                    public static final double WheelCOF = 1.0; //Coefficient of friction
                    public static final double PositionConversionFactor = 2 * WheelRadius*Math.PI/gearRatio; //Units: Meters
                    public static final double VelocityConversionFactor = PositionConversionFactor/60; //Units: Meters per second

                    //PID constants
                    public static final double kP = 0.035;
                    public static final double kI = 0.000;
                    public static final double kD = 0.0012;

                    //Feedforward constants
                    public static final double kV = 2.78;
                    public static final double kS = 0.0;
                    public static final double kA = 0.0;
                }
    
                public static class Turn {
                    public static final int CurrentLimit = 60;
                    public static final double VoltageCompensation = 12;
                    public static final double gearRatio = 12.8;
                    public static final double PositionConversionFactor = 1/gearRatio; //Units: Rotations
                    public static final double VelocityConversionFactor = PositionConversionFactor; //Units: RPM

                    //PID constants
                    public static double kP = 0.75;
                    public static final double kI = 0.0;
                    public static final double kD = 0.0001;
                }
            }
            public static enum ModuleConfig {
    
                FrontLeft(1,11,21,-0.441162109375 +0.5),
                FrontRight(2,12,22,-0.3984375 +0.5),
                RearLeft(3,13,23,-0.525146484375 ),
                RearRight(4,14,24,-0.931396484375);
    
                public final int DrivePort;
                public final int TurnPort;
                public final int EncoderPort;
                public final double EncoderOffsetRots;
    
                private ModuleConfig(int DrivePort, int TurnPort,int EncoderPort,double EncoderOffsetRots) {
                    this.DrivePort = DrivePort;
                    this.TurnPort = TurnPort;
                    this.EncoderPort = EncoderPort;
                    this.EncoderOffsetRots = EncoderOffsetRots;
                }
            }
        }
    }
}