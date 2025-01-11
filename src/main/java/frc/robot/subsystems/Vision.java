package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.apache.commons.lang3.function.TriConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants.CamConfig;
import static frc.robot.Constants.VisionConstants.Coordinates.reefCoordinates;
import static frc.robot.Constants.VisionConstants.PhotonVision.kFallbackStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kLocalizationStrategy;
import static frc.robot.Constants.VisionConstants.PhotonVision.kMultiTagDefaultStdDevs;
import static frc.robot.Constants.VisionConstants.PhotonVision.kSingleTagDefaultStdDevs;

public class Vision {

    private class CamStruct {
        public PhotonCamera camera;
        public PhotonPoseEstimator estimator;
        public CamStruct(String name,Transform3d offset) {
            this.camera = new PhotonCamera(name);
            this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), kLocalizationStrategy, offset);
            this.estimator.setMultiTagFallbackStrategy(kFallbackStrategy);
        }
    }

    //A Consumer that accepts a Pose3d and a Matrix of Standard Deviations, usually should call addVisionMeasurements() on a SwerveDrivePoseEstimator3d
    private TriConsumer<Pose2d,Double,Matrix<N3,N1>> measurementConsumer;

    HashMap<String,CamStruct> Cameras; //Hashmap of cameras used for Localization

    private Matrix<N3,N1> curStdDevs;
    
    public void registerMeasurementConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        this.measurementConsumer = consumer;
    }

    public void addCamera(String name, Transform3d offset) {
        Cameras.put(name,new CamStruct(name,offset));
    }

    public void addCamera(CamConfig config) {
        Cameras.put(config.name,new CamStruct(config.name, config.offset));
    }

    private Optional<EstimatedRobotPose> getPoseEstimate(CamStruct cam) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        List<PhotonPipelineResult> results = cam.camera.getAllUnreadResults();
        for (var res : results) {
            visionEst = cam.estimator.update(res);
            updateStdDevs(cam,visionEst,res.getTargets());
        }
        return visionEst;
    }

    /** Updates odometry with vision readings */
    public void updatePoseEstimator() {
        for (CamStruct cam : Cameras.values()) {
            Optional<EstimatedRobotPose> estimatedPose = getPoseEstimate(cam);
            if (estimatedPose.isPresent()){
                measurementConsumer.accept(
                    estimatedPose.get().estimatedPose.toPose2d(), 
                    estimatedPose.get().timestampSeconds, 
                    curStdDevs
                );
            }
        }
    }

    private void updateStdDevs(CamStruct cam, Optional<EstimatedRobotPose> estimatedPose,List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagDefaultStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagDefaultStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = cam.estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {continue;}
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagDefaultStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) {estStdDevs = kMultiTagDefaultStdDevs;}
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4){
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * @author Knivier
     * @param robotX
     * @param robotY
     * @return Coordinates of the closest april tag
     */
    public double[] closestAprilTag(int robotX, int robotY){
        double tempMinDistance = -1; // Distance away from april tag
        double aprilCoordinates[] = new double[2];


        for (int i = 0; i < reefCoordinates.length; i++){
            double aprilX = reefCoordinates[i][0];
            double aprilY = reefCoordinates[i][1];
            // Robot is 1 apriltag is 2
            double distance = Math.sqrt(Math.pow(aprilX - robotX, 2) + Math.pow(aprilY - robotY, 2));
            if (tempMinDistance == -1 || distance < tempMinDistance) {
                tempMinDistance = distance;
                aprilCoordinates[0] = aprilX;
                aprilCoordinates[1] = aprilY;
            }
        }
        return aprilCoordinates; //yay
    }


    public double[] leftCoordinateCentering(double x, double y){

        
        return null;
    }

    public double[] rightCoordinateCentering(double x, double y){

        return null;
    }


}
