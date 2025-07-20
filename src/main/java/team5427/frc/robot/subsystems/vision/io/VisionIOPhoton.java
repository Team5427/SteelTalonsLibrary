package team5427.frc.robot.subsystems.vision.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import team5427.frc.robot.subsystems.Swerve.SwerveSubsystem;
import team5427.frc.robot.subsystems.vision.VisionConstants;
import team5427.lib.detection.tuples.Tuple2Plus;

public class VisionIOPhoton implements VisionIO {

  private PhotonCamera cam;

  public Matrix<N3, N1> stddev;

  private PhotonPoseEstimator photonPoseEstimator;

  private Transform3d cameraOffset;

  private Supplier<Pose2d> getReferencePose;

  private Supplier<Tuple2Plus<Double, Rotation2d>> getHeadingData;

  public VisionIOPhoton(
      String cameraName,
      Transform3d cameraTransform,
      Supplier<Pose2d> getReferencePose,
      Supplier<Tuple2Plus<Double, Rotation2d>> getHeadingData) {
    cam = new PhotonCamera(cameraName);

    photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, cameraTransform);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CONSTRAINED_SOLVEPNP);
    this.cameraOffset = cameraTransform;
    this.getReferencePose = getReferencePose;
    this.getHeadingData = getHeadingData;

    // flush any old results from previous results
    this.cam.getAllUnreadResults();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = cam.isConnected();
    List<PhotonPipelineResult> results = cam.getAllUnreadResults();
    List<PoseObservation> obs = new LinkedList<PoseObservation>();
    photonPoseEstimator.setReferencePose(getReferencePose.get());
    photonPoseEstimator.addHeadingData(getHeadingData.get().r, getHeadingData.get().t);

    for (int i = results.size() - 1; i >= 0; i--) {

      if (results.get(i).multitagResult.isPresent()) {
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(results.get(i));
        double totalTagDistance = 0.0;
        Set<Short> tagIdSet = new HashSet<>();
        for (PhotonTrackedTarget target : results.get(i).targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
          tagIdSet.add((short) target.fiducialId);
        }
        inputs.tagIds = new int[tagIdSet.size()];
        for (int j = 0; j < tagIdSet.size(); j++) {
          inputs.tagIds[j] = (Short) tagIdSet.toArray()[j];
        }

        obs.add(
            new PoseObservation(
                estimatedPose.get().timestampSeconds,
                estimatedPose.get().estimatedPose,
                results.get(i).multitagResult.get().estimatedPose.ambiguity,
                results.get(i).multitagResult.get().fiducialIDsUsed.size(),
                totalTagDistance / results.get(i).targets.size(),
                results.get(i).getBestTarget().getYaw(),
                results.get(i).getBestTarget().getPitch(),
                PoseObservationType.PHOTONVISION_MULTI_TAG));
      } else {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(results.get(i));
        photonPoseEstimator.addHeadingData(
            Timer.getTimestamp(), SwerveSubsystem.getInstance().getGyroRotation());
        List<PhotonTrackedTarget> targets = results.get(i).getTargets();
        for (PhotonTrackedTarget target : targets) {
          Pose3d robotPose = new Pose3d();
          if (VisionConstants.kAprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            robotPose =
                PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    VisionConstants.kAprilTagLayout.getTagPose(target.getFiducialId()).get(),
                    photonPoseEstimator.getRobotToCameraTransform().inverse());
          }
          obs.add(
              new PoseObservation(
                  pose.get().timestampSeconds,
                  robotPose,
                  target.getPoseAmbiguity(),
                  1,
                  target.getBestCameraToTarget().getTranslation().getNorm(),
                  target.getYaw(),
                  target.getPitch(),
                  PoseObservationType.PHOTONVISION_SINGLE_TAG));
        }
      }
      // List<PoseObservation> temp = Arrays.asList(inputs.poseObservations);
      // temp = new LinkedList<PoseObservation>(temp);
      // temp.addAll(obs);

      inputs.poseObservations = new PoseObservation[obs.size()];

      for (int b = 0; b <= obs.size() - 1; b++) {
        inputs.poseObservations[b] = obs.get(b);
      }
    }
  }

  @Override
  public void applyCameraTransformation(Transform3d transformation) {
    cameraOffset = transformation;
    photonPoseEstimator.setRobotToCameraTransform(transformation);
  }

  @Override
  public void setStdDev(Matrix<N3, N1> stddev) {
    this.stddev = stddev;
  }
}
