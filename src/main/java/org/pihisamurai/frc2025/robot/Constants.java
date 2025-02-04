// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pihisamurai.frc2025.robot;

import static org.pihisamurai.frc2025.robot.Constants.DriveConstants.moduleTranslations;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

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
            LEFT_ARDUCAM("FLMOD_CAM", DriveConstants.moduleTranslations[0].getX(), moduleTranslations[0].getY(), Units.inchesToMeters(14), 0, 0, 0),
            RIGHT_ARDUCAM("FRMOD_CAM", DriveConstants.moduleTranslations[2].getX(), moduleTranslations[0].getY(), Units.inchesToMeters(14), 0, 0, 0);

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

    }
    

    public static class Akit {
        //0 = real, 1 = Sim, 2 = replay
        //If statements that evaluate this constant expression are used to implement C-style 
        public static final int currentMode = 0;
    }
      
    /** Contains data about the field */
    public static class FieldConstants {
        private static final double branchOffset = Units.inchesToMeters(6.469);
        private static final Transform2d leftBranchTransform = new Transform2d(0.0, -branchOffset, Rotation2d.kZero);
        private static final Transform2d rightBranchTransform = new Transform2d(0.0, branchOffset, Rotation2d.kZero);
        /**
         * @description Provides coordinates for april tags
         * @description See https://docs.google.com/spreadsheets/d/1mz5djBDrFm8Ro_M04Yq4eea92x4Xyj_pqlt54wsXnxA/edit?usp=sharing
         */
        public enum ReefFace {
            // IMPORTANT: Fudge factors are always positive and should be in meters (use the Units.inchesToMeters() method)

            //Blue Reef
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, null, null),
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, null, null),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, null, null),
            BLU_REEF_GH(21, 5.321046, 4.025900, 0.0, null, null),
            BLU_REEF_IJ(20, 4.904740, 4.745482, 60.0, null, null),
            BLU_REEF_KL(19, 4.073906, 4.745482, 120.0, null, null),

            //Red Reef
            RED_REEF_AB(7, 13.890498, 4.025900, 0.0, null, null),
            RED_REEF_CD(8, 13.474446, 4.745482, 60., null, null),
            RED_REEF_EF(9, 12.643358, 4.745482, 120.0, null, null),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, null, null),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, null, null),
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, null, null);


            public final Double leftBranchFudgeTransform;
            public final Double rightBranchFudgeTransform;
            public final Pose2d leftBranch;
            public final Pose2d rightBranch;
            public final Pose2d AprilTag;
            public final int AprilTagID;

            //AT stands for AprilTag
            private ReefFace(int AprilTagID, double AT_x, double AT_y, double AT_theta, Double leftBranchFudgeTransform, Double rightBranchFudgeTransform) {
                this.AprilTagID = AprilTagID;
                this.AprilTag = new Pose2d(AT_x, AT_y, Rotation2d.fromDegrees(AT_theta));
                this.leftBranchFudgeTransform = leftBranchFudgeTransform;
                this.rightBranchFudgeTransform = rightBranchFudgeTransform;

                if(this.leftBranchFudgeTransform == null) {
                    this.leftBranch = AprilTag.transformBy(leftBranchTransform);
                } else {
                    this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
                }
                
                if(this.rightBranchFudgeTransform == null) {
                    this.rightBranch = AprilTag.transformBy(rightBranchTransform);
                } else {
                    this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
                }
            }
        }

        //Generic rotation-agnostic points of interest
        public enum PointOfInterest {
            BLU_REEF(4.487,4.010),
            RED_REEF(13.062,4.010);

            public final Translation2d position;
            private PointOfInterest(double xMeters, double yMeters) {
                this.position = new Translation2d(xMeters,yMeters);
            }
        }

        //Poses of interest
        public enum PoseOfInterest {
            BLU_PROCESSOR(5.973318,-0.00381,90), //Taken from April Tag coordinates
            RED_PROCESSOR(11.56081,	8.05561,	270), //Taken from April Tag coordinates
            BLU_RIGHT_STATION(Units.inchesToMeters(33.51),Units.inchesToMeters(25.80),55),
            BLU_LEFT_STATION(Units.inchesToMeters(33.51),Units.inchesToMeters(291.20),305),
            RED_RIGHT_STATION(Units.inchesToMeters(657.37),Units.inchesToMeters(291.20),-125),
            RED_LEFT_STATION(Units.inchesToMeters(657.37),Units.inchesToMeters(25.80),125);

            public final Pose2d pose;

            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) {
                this.pose = new Pose2d(xMeters,yMeters,Rotation2d.fromDegrees(omegaDeg));
            }
        }
    }

    public static class DriveConstants {
        public static final int odometryFrequencyHz = 250;
        public static final double wheelBase = Units.inchesToMeters(27.5); //Meters
        public static final double trackWidth = Units.inchesToMeters(19.5); //Meters
        public static final double maxTranslationSpeed = Units.feetToMeters(13); //meters per second
        public static final double maxRotationSpeed = 5; // Radians Per Second
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

        public static enum TeleopDriveMode {
            kClosedLoopFieldOriented,
            kClosedLoopChassisOriented,
            kOpenLoopFieldOriented,
            kOpenLoopChassisOriented
        }

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
                            DCMotor.getNEO(1).withReduction(ModuleConstants.Common.Drive.gearRatio),
                            ModuleConstants.Common.Drive.CurrentLimit,
                            1
                    ),
                    moduleTranslations
            );

            public static final PathConstraints pathConstraints = new PathConstraints(
                MetersPerSecond.of(5), 
                MetersPerSecondPerSecond.of(5), 
                RadiansPerSecond.of(5), 
                RadiansPerSecondPerSecond.of(5)
            );
        

            public static class PIDControl {
                public static class Trans {
                    public static final double kP = 5.0;
                    public static final double kI = 0.0;
                    public static final double kD = 0.0;
                }

                public static class Rot {
                    public static final double kP = 6.0;
                    public static final double kI = 0.0;
                    public static final double kD = 0.00;
                }
            }
        }

        public static class ModuleConstants {
            public static class Common {
                public static class Drive {
                    public static final int CurrentLimit = 60;
                    public static final double gearRatio = 6.75;
                    public static final double VoltageCompensation = 12;
                  
                    public static final double MaxModuleSpeed = 5.0; //Maximum attainable module speed (m/s)
                    public static final double WheelRadius = 0.0508; //Meters
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
                    public static final double MoI = 0.010; //Placeholder, run sysID characterization routine to find actual value
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
                    public static final double MoI = 0.010; //Placeholder
                }
            }

            public static enum ModuleConfig {

                FrontLeft(1, 11, 21, -0.441162109375 + 0.5),
                FrontRight(2, 12, 22, -0.3984375 + 0.5),
                RearLeft(3, 13, 23, -0.525146484375 + 0.5),
                RearRight(4, 14, 24, -0.931396484375 + 0.5);

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
