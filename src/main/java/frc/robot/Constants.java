// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

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
            public static final double kControllerDeadband = 0.15;
            public static final double kControllerTriggerThreshold = 0.7;
        }
    }

    public static class VisionConstants {
        public static enum CamConfig {
            LEFT_ARDUCAM("PLACEHOLDER_0", 0, 0, 0, 0, 0, 0),
            RIGHT_ARDUCAM("PLACEHOLDER_1", 0, 0, 0, 0, 0, 0);

            public final String name;
            public final double x;
            public final double y;
            public final double z;
            public final double roll;
            public final double pitch;
            public final double yaw;
            public final Transform3d offset;

            private CamConfig(String name, double x, double y, double z, double roll, double pitch, double yaw) {
                this.name = name;
                this.x = x;
                this.y = y;
                this.z = z;
                this.roll = roll;
                this.pitch = pitch;
                this.yaw = yaw;
                this.offset = new Transform3d(x, y, z, new Rotation3d(roll, pitch, yaw));
            }
        }

        public static class PhotonVision {
            public static final PoseStrategy kLocalizationStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
            public static final PoseStrategy kFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
            public static final Matrix<N3, N1> kSingleTagDefaultStdDevs = VecBuilder.fill(4, 4, 8);
            public static final Matrix<N3, N1> kMultiTagDefaultStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }

    public static class Akit {
        //0 = real, 1 = Sim, 2 = replay
        //If statements that evaluate this constant expression are used to implement C-style 
        public static final int currentMode = 1;
    }
      
        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public static class Coordinates {
            public static final double[][] reefAprilCoordinates = new double[][]{
                    {4.073906, 3.306318}, // 17
                    {3.6576, 4.0259}, // 18
                    {4.073906, 4.745482}, // 19
                    {4.90474, 4.74582}, // 20
                    {5.321046, 4.0259}, // 21
                    {4.90474, 3.306318}, // 22 - Sect 1 (BLUE) ends on sheet
                    {13.474446, 3.306318}, // 6
                    {13.890498, 4.0259}, // 7
                    {13.474446, 4.745482}, // 8
                    {12.643358, 4.745482}, // 9
                    {12.227306, 4.0259}, // 10
                    {12.643358, 3.306318} // 11 - Sect 2 (RED) ends
            };

            public static final double[] reefAprilAngles = new double[]{
                240, // 17
                180, // 18
                120, // 19
                60, // 20
                0, // 21
                300, // 22
                300, // 6
                0, // 7
                60, // 8
                120, // 9
                180, // 10
                240 // 11
            };
            
            /**
             * Coordinates of the left branches on the reef, with indexes that correspond to those of reefAprilCoordinates.
             */
            public static final double[][] reefLeftBranchCoordinates = new double[][]{
                {3.93, 3.4}, // 17 
                {3.658, 4.18}, // 18 
                {4.19, 4.85}, // 19 
                {5.06, 4.69}, // 20 
                {5.321, 3.85}, // 21 
                {4.78, 3.22}, // 22 
                {15.35, 3.21}, // 6 
                {13.91, 3.86}, // 7 
                {13.64, 4.69}, // 8 
                {12.78, 4.83}, // 9 
                {12.22, 4.19}, // 10 
                {12.49, 3.36} // 11
            };
            
            /**
             * Coordinates of the right branches on the reef, with indexes that correspond to those of reefAprilCoordinates
             */
            public static final double[][] reefRightBranchCoordinates = new double[][]{
                {4.21, 3.2}, // 17 
                {3.658, 3.86}, // 18 
                {3.92, 4.67}, // 19 
                {4.78, 4.84}, // 20 
                {5.321, 4.17}, // 21 
                {5.06, 3.34}, // 22 
                {13.64, 3.34}, // 6 
                {13.91, 4.19}, // 7 
                {13.36, 4.83}, // 8 
                {12.51, 4.68}, // 9 
                {12.22, 3.85}, // 10
                {12.79, 3.21} // 11
            };
        }
    }

    public static class DriveConstants {
        public static final int odometryFrequencyHz = 250;
        public static final double wheelBase = Units.inchesToMeters(27.5); //Meters
        public static final double trackWidth = Units.inchesToMeters(19.5); //Meters
        public static final double maxTranslationSpeed = Units.feetToMeters(1); //meters per second
        public static final double maxRotationSpeed = 0.5; // Radians Per Second
        public static final double singleClutchTranslationFactor = 0.5;
        public static final double singleClutchRotationFactor = 0.5;
        public static final double doubleClutchTranslationFactor = 0.3;
        public static final double doubleClutchRotationFactor = 0.35;
        //public static final double wheelRadius = 0.0508; //Meters
        public static final Translation2d[] moduleTranslations = new Translation2d[]{
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
                  
                    public static final double MaxModuleSpeed = 5.0; //Maximum attainable module speed
                    public static final double WheelRadius = Units.inchesToMeters(2); //Meters
                    public static final double WheelCOF = 1.0; //Coefficient of friction
                    public static final double PositionConversionFactor = 2*WheelRadius*Math.PI/gearRatio; //Units: Meters
                    public static final double VelocityConversionFactor = PositionConversionFactor / 60; //Units: Meters per second

                    //PID constants
                    public static final double kP = 0.0015;
                    public static final double kI = 0.000;
                    public static final double kD = 0.0000;

                    // Feedforward constants
                    public static final double kV = 2.78;
                    public static final double kS = 0.0;
                    public static final double kA = 0.0;

                    //Physical constants
                    public static final double MoI = 0.025; //Placeholder, run sysID characterization routine to find actual value
                }

                public static class Turn {
                    public static final int CurrentLimit = 60;
                    public static final double VoltageCompensation = 12;
                    public static final double gearRatio = 12.8;
                    public static final double PositionConversionFactor = 1 / gearRatio; // Units: Rotations
                    public static final double VelocityConversionFactor = PositionConversionFactor; // Units: RPM


                    //PID constants
                    public static final double kP = 1.05;
                    public static final double kI = 0.0;
                    public static final double kD = 0.0;

                    //Feedforward constants
                    public static final double kV = 2.78;
                    public static final double kS = 0.0;
                    public static final double kA = 0.0;

                    //Physical constants
                    public static final double MoI = 0.025; //Placeholder
                }
            }

            public static enum ModuleConfig {

                FrontLeft(1, 11, 21, -0.441162109375 + 0.5),
                FrontRight(2, 12, 22, -0.3984375 + 0.5),
                RearLeft(3, 13, 23, -0.525146484375),
                RearRight(4, 14, 24, -0.931396484375);

                public final int DrivePort;
                public final int TurnPort;
                public final int EncoderPort;
                public final double EncoderOffsetRots;

                private ModuleConfig(int DrivePort, int TurnPort, int EncoderPort, double EncoderOffsetRots) {
                    this.DrivePort = DrivePort;
                    this.TurnPort = TurnPort;
                    this.EncoderPort = EncoderPort;
                    this.EncoderOffsetRots = EncoderOffsetRots;
                }
            }
        }
    }
}
