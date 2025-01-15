package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.VisionConstants.ReefFace;

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
