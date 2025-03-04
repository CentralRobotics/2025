package frc.robot.subsystems.swervedrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;

public class Camera {
    private PhotonCamera camera;
    private PhotonCameraSim simCamera;
    private Matrix<N3, N1> singleTagStdDevs;
    private Matrix<N3, N1> multiTagStdDevsMatrix;
    private SimCameraProperties simProperties;
    private PhotonPoseEstimator poseEstimator;
    private Transform3d robotToCam;
    private StructPublisher<Pose2d> posePublisher;
    private StructArrayPublisher<Pose3d> trackedTargetsPublisher;
    private StructArrayPublisher<Translation2d> trackedCornersPublisher;
    private List<BooleanSupplier> disableConditions;

    public Camera(String name, Translation3d robotToCamTranslation, Rotation3d robotToCamRotation, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix, List<BooleanSupplier> disableConditions) {
        camera = new PhotonCamera(name);
        posePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision/" + name + "/Estimated Pose", Pose2d.struct).publish();
        trackedTargetsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/" + name + "/Tracked Targets", Pose3d.struct).publish();
        trackedCornersPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/" + name + "/Corners", Translation2d.struct).publish();

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevsMatrix = multiTagStdDevsMatrix;

        simProperties = new SimCameraProperties();
        simProperties.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        simProperties.setCalibError(0.25, 0.08);
        simProperties.setFPS(30);
        simProperties.setAvgLatencyMs(35);
        simProperties.setLatencyStdDevMs(5);

        simCamera = new PhotonCameraSim(camera, simProperties);
        robotToCam = new Transform3d(robotToCamTranslation, robotToCamRotation);

        poseEstimator = new PhotonPoseEstimator(Constants.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        simCamera.enableDrawWireframe(true);

        this.disableConditions = disableConditions;
    }

    public Camera(String name, Translation3d robotToCamTranslation, Rotation3d robotToCamRotation, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
        this(name, robotToCamTranslation, robotToCamRotation, singleTagStdDevs, multiTagStdDevsMatrix, new ArrayList<>());
    }

    public void addCondition(BooleanSupplier condition) {
        disableConditions.add(condition);
    }

    public void addCondition(List<BooleanSupplier> conditions) {
        disableConditions.addAll(conditions);
    }

    private boolean shouldDisable() {
        for(BooleanSupplier conditon : disableConditions) {
            if(conditon.getAsBoolean()) return true;
        }
        return false;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public PhotonCameraSim getSimCamera() {
        return simCamera;
    }

    public Matrix<N3, N1> getSingleTagStdDevs() {
        return singleTagStdDevs;
    }

    public Matrix<N3, N1> getMultiTagStdDevsMatrix() {
        return multiTagStdDevsMatrix;
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public SimCameraProperties getSimProperties() {
        return simProperties;
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    public Optional<EstimatedRobotPose> updatePoseEstimator(PhotonPipelineResult result) {
        if(shouldDisable()) return Optional.empty();
        return poseEstimator.update(result);
    }

    public List<PhotonPipelineResult> getAllUnusedResults() {
        return camera.getAllUnreadResults();
    }

    public StructPublisher<Pose2d> getPosePublisher() {
        return posePublisher;
    }

    public StructArrayPublisher<Pose3d> getTrackedTargetsPublisher() {
        return trackedTargetsPublisher;
    }

    public StructArrayPublisher<Translation2d> getTrackedCornersPublisher() {
        return trackedCornersPublisher;
    }
}
