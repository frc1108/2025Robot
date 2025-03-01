package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ReefVisionConstants;


@Logged
public class Vision extends SubsystemBase {
    private final PhotonCamera reefCamera, bargeCamera;    
    private final PhotonPoseEstimator reefCameraPoseEstimator, bargeCameraPoseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    private final BiConsumer<Pose2d, Double> consumer;
    private final DriveSubsystem drive;

    public Vision(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive) throws IOException{

        // Upper and lower cameras are used
        reefCamera = new PhotonCamera(Constants.ReefVisionConstants.kCameraName);
        bargeCamera = new PhotonCamera(Constants.BargeVisionConstants.kCameraName);

        // Field layout is loaded
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);

        reefCameraPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.ReefVisionConstants.kCameraOffset);

        bargeCameraPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            Constants.BargeVisionConstants.kCameraOffset);

        this.consumer = consumer;
        this.drive = drive;
    }

    private boolean isReefCameraConnected() {
        return reefCamera.isConnected();
    }

    private boolean isBargeCameraConnected() {
        return bargeCamera.isConnected();
    }

    @Override
    public void periodic(){
        if (!isReefCameraConnected() && !isBargeCameraConnected()) 
            return;

        PhotonPipelineResult reefPipelineResult = new PhotonPipelineResult();
        PhotonPipelineResult bargePipelineResult = new PhotonPipelineResult();
        List<PhotonTrackedTarget> badTargets = new ArrayList<>();

        if (isReefCameraConnected())
         {
            reefPipelineResult = reefCamera.getLatestResult();
            boolean hasReefTargets = reefPipelineResult.hasTargets();
            if (!hasReefTargets)
                return;

            List<PhotonTrackedTarget> badReefTargets = new ArrayList<>();
            for(PhotonTrackedTarget reefTarget : reefPipelineResult.targets){
                var reefTagPose = fieldLayout.getTagPose(reefTarget.getFiducialId());
                var distanceToReefTag = PhotonUtils.getDistanceToPose(drive.getPose(),reefTagPose.get().toPose2d());
                if(reefTarget.getPoseAmbiguity()>0.35 || distanceToReefTag > ReefVisionConstants.kMaxDistanceMeters){  // Changed from 0.5 ambiguity
                    badReefTargets.add(reefTarget);
                }
            }
            reefPipelineResult.targets.removeAll(badReefTargets);
         }

         if (isBargeCameraConnected())
         {
            bargePipelineResult = reefCamera.getLatestResult();
            boolean hasBargeTargets = bargePipelineResult.hasTargets();
            if (!hasBargeTargets)
                return;

            List<PhotonTrackedTarget> badBargeTargets = new ArrayList<>();
            for(PhotonTrackedTarget bargeTarget : bargePipelineResult.targets){
                var bargeTagPose = fieldLayout.getTagPose(bargeTarget.getFiducialId());
                var distanceToBargeTag = PhotonUtils.getDistanceToPose(drive.getPose(),bargeTagPose.get().toPose2d());
                if(bargeTarget.getPoseAmbiguity()>0.35 || distanceToBargeTag > ReefVisionConstants.kMaxDistanceMeters){  // Changed from 0.5 ambiguity
                    badBargeTargets.add(bargeTarget);
                }
            }
            bargePipelineResult.targets.removeAll(badBargeTargets);
         }
        

        PhotonPipelineResult totalPipelineResult = new PhotonPipelineResult();
        totalPipelineResult.targets.addAll(bargePipelineResult.targets);
        totalPipelineResult.targets.addAll(reefPipelineResult.targets);
        
        Optional<EstimatedRobotPose> reefPoseResult = reefCameraPoseEstimator.update(totalPipelineResult);
        boolean posePresent = reefPoseResult.isPresent();
        if (!posePresent)
            return;

        EstimatedRobotPose estimatedPose = reefPoseResult.get();

        consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }


}