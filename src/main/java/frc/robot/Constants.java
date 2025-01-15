// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {

            //Blue Reef
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, 3.930, 3.400, 4.210, 3.200),
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, 3.658, 4.180, 3.658, 3.860),
            BLU_REEF_LK(19, 4.073906, 4.745482, 120.0, 4.190, 4.850, 3.920, 4.670),
            BLU_REEF_JI(20, 4.904740, 4.745482, 60.0, 5.060, 4.690, 4.780, 4.840),
            BLU_REEF_HG(21, 5.321046, 4.025900, 0.0, 5.321, 3.850, 5.321, 4.170),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, 4.780, 3.220, 5.060, 3.340),

            //Red Reef
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, 15.350, 3.210, 13.640, 3.340),
            RED_REEF_BA(7, 13.890498, 4.025900, 0.0, 13.910, 3.860, 13.910, 4.190),
            RED_REEF_DC(8, 13.474446, 4.745482, 60.0, 13.640, 4.690, 13.360, 4.830),
            RED_REEF_FE(9, 12.643358, 4.745482, 120.0, 12.780, 4.830, 12.510, 4.680),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, 12.220, 4.190, 12.220, 3.850),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, 12.490, 3.360, 12.790, 3.210);
        

            public final Translation2d leftBranch;
            public final Translation2d rightBranch;
            public final Pose2d AprilTag;
            public final int AprilTagID;
        
            private ReefFace(int AprilTagID, double AT_x, double AT_y, double AT_theta, double L_x, double L_y, double R_x, double R_y) {
                this.AprilTagID = AprilTagID;
                this.AprilTag = new Pose2d(AT_x, AT_y, Rotation2d.fromDegrees(AT_theta));
                this.leftBranch = new Translation2d(L_x, L_y);
                this.rightBranch = new Translation2d(R_x, R_y);
            }
        }
    }
    

    public static class Akit {
        //0 = real, 1 = Sim, 2 = replay
        //If statements that evaluate this constant expression are used to implement C-style 
        public static final int currentMode = 1;
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
                    0.05,
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
