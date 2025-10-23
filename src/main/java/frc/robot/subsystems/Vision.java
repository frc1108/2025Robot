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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReefVisionConstants;


@Logged
public class Vision extends SubsystemBase {
    private final PhotonCamera photonCamera;    
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    private final BiConsumer<Pose2d, Double> consumer;
    @NotLogged private final DriveSubsystem drive;
    private Pose3d estimated3dPose;

    public Vision(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive, String photonCameraName, Transform3d cameraOffset) throws IOException{

        // Upper and lower cameras are used
        photonCamera = new PhotonCamera(photonCameraName);

        // Field layout is loaded
        fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        //fieldLayout = new AprilTagFieldLayout("/home/lvuser/deploy/fieldmaps/2025-reefscape-welded.json");


        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraOffset);

        estimated3dPose = new Pose3d();

        this.consumer = consumer;
        this.drive = drive;
    }

     @Override
    public void periodic(){
        if (!photonCamera.isConnected()) 
            return;

          var pipelineResult = photonCamera.getLatestResult();
          boolean hasTargets = pipelineResult.hasTargets();
            if (!hasTargets)
                return;
          
          List<PhotonTrackedTarget> badTargets = new ArrayList<>();
            for(PhotonTrackedTarget target : pipelineResult.targets){
                var tagPose = fieldLayout.getTagPose(target.getFiducialId());
                var distanceTotag = PhotonUtils.getDistanceToPose(drive.getPose(),tagPose.get().toPose2d());
                if(target.getPoseAmbiguity()>0.35 || distanceTotag > ReefVisionConstants.kMaxDistanceMeters){  // Changed from 0.5 ambiguity
                    badTargets.add(target);
                }
            }
            pipelineResult.targets.removeAll(badTargets);
             
          Optional<EstimatedRobotPose> poseResult = poseEstimator.update(pipelineResult);
          boolean posePresent = poseResult.isPresent();
          if (!posePresent)
            return;

          EstimatedRobotPose estimatedPose = poseResult.get();

          estimated3dPose = estimatedPose.estimatedPose;
          consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }

    public Pose3d getEstimated3dPose() {
        return estimated3dPose;
    }


}