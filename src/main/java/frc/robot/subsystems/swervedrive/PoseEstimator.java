package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Robot;
import swervelib.SwerveDrive;

public class PoseEstimator {
    private VisionSystemSim visionSim;
    private ArrayList<Camera> cameras;
    private Supplier<Pose2d> getSimDrivetrainPose;

    public PoseEstimator(Supplier<Pose2d> getSimPose) {
        visionSim = new VisionSystemSim("Vision Sim");
        visionSim.addAprilTags(Constants.fieldLayout);
        this.cameras = new ArrayList<Camera>();
        this.getSimDrivetrainPose = getSimPose;
    }

    public void addCamera(Camera camera) {
        cameras.add(camera);
        visionSim.addCamera(camera.getSimCamera(), camera.getRobotToCam());
    }

    public void addCameras(Camera... cameras) {
        for(Camera camera : cameras) {
            addCamera(camera);
        }
        
    }

    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if(Robot.isSimulation()) visionSim.update(getSimDrivetrainPose.get());
        for(Camera camera : cameras) {
            var results = camera.getAllUnusedResults();
            if(!results.isEmpty()) {
                PhotonPipelineResult latestResult = results.get(results.size() - 1);

                Optional<EstimatedRobotPose> estPose = camera.updatePoseEstimator(latestResult);
                if(estPose.isPresent()) {
                    EstimatedRobotPose pose = estPose.get();
                    swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, getEstimationStdDevs(camera, pose));
                    camera.getPosePublisher().set(pose.estimatedPose.toPose2d());
                }

                if(latestResult.hasTargets()) {
                    Pose3d[] posesArray = new Pose3d[latestResult.targets.size()];
                    Translation2d[] corners = new Translation2d[latestResult.targets.size() * 4];
                    int cornerIndex = 0;
                    for(int i = 0; i < posesArray.length; i++) {
                        PhotonTrackedTarget target = latestResult.targets.get(i);
                        posesArray[i] = Constants.fieldLayout.getTagPose(target.getFiducialId()).get();
                        corners[cornerIndex] = new Translation2d(target.getDetectedCorners().get(0).x, target.getDetectedCorners().get(0).y);
                        corners[cornerIndex + 1] = new Translation2d(target.getDetectedCorners().get(1).x, target.getDetectedCorners().get(1).y);
                        corners[cornerIndex + 2] = new Translation2d(target.getDetectedCorners().get(2).x, target.getDetectedCorners().get(2).y);
                        corners[cornerIndex + 3] = new Translation2d(target.getDetectedCorners().get(3).x, target.getDetectedCorners().get(3).y);
                        cornerIndex += 4;
                    }
                    camera.getTrackedTargetsPublisher().set(posesArray);
                    camera.getTrackedCornersPublisher().set(corners);

                } else {
                    camera.getTrackedTargetsPublisher().set(new Pose3d[0]);
                    camera.getTrackedCornersPublisher().set(new Translation2d[0]);
                }
            }
        }
    }

    public Matrix <N3, N1> getEstimationStdDevs(Camera camera, EstimatedRobotPose poseEst) {
        var estStdDevs = camera.getSingleTagStdDevs();
        var targets = poseEst.targetsUsed;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt: targets) {
            var tagPose = camera.getPoseEstimator().getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            numTags++;
            avgDist += PhotonUtils.getDistanceToPose(poseEst.estimatedPose.toPose2d(), tagPose.get().toPose2d());
        }
        if (numTags == 0) {
            return estStdDevs;
        }
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = camera.getMultiTagStdDevsMatrix();
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        return estStdDevs;
    }
}
