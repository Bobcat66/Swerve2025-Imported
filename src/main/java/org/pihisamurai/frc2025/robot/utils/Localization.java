package org.pihisamurai.frc2025.robot.utils;

import java.util.ArrayList;
import java.util.List;

import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PoseOfInterest;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.ReefFace;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class containing utility functions for localization
 * 
 * @author Tejas Gupta
 * @author David Loh
 * @author Jesse Kane
 */
public final class Localization {

    /**
     * @author Jesse Kane
     * @param robotPose
     * @return the closest Reef Face
     */
    public static ReefFace getClosestReefFace(Pose2d robotPose){
        double closestDistance = Double.MAX_VALUE; // Distance away from april tag
        ReefFace closestFace = null;

        for (ReefFace face: ReefFace.values()){
            double distance = robotPose.getTranslation().getDistance(face.AprilTag.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestFace = face;
            }
        }

        return closestFace;
    }

    /** 
     * Returns the pose of the nearest coral station 
     * 
     * @author Jesse Kane
     */
    public static Pose2d getClosestCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getCoralStationPoses());
    }

    public static Pose2d getClosestBlueCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getBlueCoralStationPoses());
    }

    public static Pose2d getClosestRedCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getRedCoralStationPoses());
    }

    /** Returns a list of all coral stations */
    public static List<Pose2d> getCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_OPPOSITE.pose);
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_PROCESSOR.pose);
        poseList.add(PoseOfInterest.RED_CORAL_STATION_PROCESSOR.pose);
        poseList.add(PoseOfInterest.RED_CORAL_STATION_OPPOSITE.pose);
        return poseList;
    }

    /** Returns a list of red coral stations */
    public static List<Pose2d> getRedCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.RED_CORAL_STATION_PROCESSOR.pose);
        poseList.add(PoseOfInterest.RED_CORAL_STATION_OPPOSITE.pose);
        return poseList;
    }

    /** Returns a list of blue coral stations */
    public static List<Pose2d> getBlueCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_OPPOSITE.pose);
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_PROCESSOR.pose);
        return poseList;
    }

    private static ReefFace getReefFromAprilTagID(int AprilTagID){
        for (ReefFace face: ReefFace.values()) {
            if (face.AprilTagID == AprilTagID) {
                return face;
            }
        }
        return null;
    }

    /**
     * @param robotX
     * @param robotY
     * @return Index of the closest reef april tag
     * @deprecated replaced by {@link #getClosestReefFace()}
     */
    @Deprecated
    public static int getClosestAprilTagIndex(double robotX, double robotY){

        double tempMinDistance = Double.MAX_VALUE; // Distance away from april tag
        int aprilIndex = -1; // index of that april tag

        for (ReefFace face: ReefFace.values()){
            double distance = (new Translation2d(robotX,robotY)).getDistance(face.AprilTag.getTranslation());
            if (distance < tempMinDistance) {
                tempMinDistance = distance;
                aprilIndex = face.AprilTagID;
            }
        }
        return aprilIndex; //yay
    }
    
    /**
     * @param robotX
     * @param robotY
     * @return The coordinates of the closest AprilTag
     * @deprecated replaced by {@link #getClosestReefFace()}
     */
    @Deprecated
    public static double[] getClosestAprilTagCoordinates(double robotX, double robotY) {
        int index = getClosestAprilTagIndex(robotX,robotY);
        ReefFace face = getReefFromAprilTagID(index);
        double coords[] = {face.AprilTag.getX(),face.AprilTag.getY()};
        return coords;
    }

    /**
     * @param robotX x-pose of the robot
     * @param robotY y-pose of the robot
     * @return The location of the branch to the left of the nearest reef April Tag.
     * @deprecated replaced by {@link #getClosestReefFace()}
     */
    @Deprecated
    public static double[] getClosestLeftBranchCoordinates(double robotX, double robotY){
        int index = getClosestAprilTagIndex(robotX, robotY);
        ReefFace face = getReefFromAprilTagID(index);
        double coords[] = {face.leftBranch.getX(),face.leftBranch.getY()};
        return coords;
    }

    /**
     * @author Tejas Gupta
     * @param robotX x-pose of the robot
     * @param robotY y-pose of the robot
     * @return The location of the branch to the right of the nearest reef April Tag
     * @deprecated replaced by {@link #getClosestReefFace()}
     */
    @Deprecated
    public static double[] getClosestRightBranchCoordinate(double robotX, double robotY){
        int index = getClosestAprilTagIndex(robotX, robotY);
        ReefFace face = getReefFromAprilTagID(index);
        double coords[] = {face.rightBranch.getX(),face.rightBranch.getY()};
        return coords;
    }
}
