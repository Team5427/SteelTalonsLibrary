package team5427.frc.robot.subsystems.vision.io;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import team5427.frc.robot.subsystems.vision.VisionConstants;
import team5427.lib.detection.tuples.Tuple2Plus;

public class VisionIOPhotonSim implements VisionIO {

  private PhotonCameraSim sim;

  private VisionSystemSim visionSystemSim;

  private PhotonPoseEstimator photonPoseEstimator;
  public Matrix<N3, N1> stddev;
  Supplier<Pose2d> getReferencePose;

  Supplier<Tuple2Plus<Double, Rotation2d>> getHeadingData;

  // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
  //         AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape),
  // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  //         VisionConstants.swerveCamTransform);

  public VisionIOPhotonSim(
      String cameraName,
      Transform3d cameraTransform,
      Supplier<Pose2d> getReferencePose,
      Supplier<Tuple2Plus<Double, Rotation2d>> getHeadingData) {
    sim = new PhotonCameraSim(new PhotonCamera(cameraName));
    sim.setMaxSightRange(VisionConstants.kCameraMaxRange.in(Meter));
    sim.enableProcessedStream(true);
    visionSystemSim = new VisionSystemSim("Argo Cam " + cameraName);
    visionSystemSim.addAprilTags(VisionConstants.kAprilTagLayout);
    visionSystemSim.addCamera(sim, cameraTransform);
    this.getHeadingData = getHeadingData;
    this.getReferencePose = getReferencePose;
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.kAprilTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, cameraTransform);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = sim.getCamera().isConnected();
    photonPoseEstimator.setReferencePose(getReferencePose.get());
    visionSystemSim.update(getReferencePose.get());
    photonPoseEstimator.addHeadingData(getHeadingData.get().r, getHeadingData.get().t);
    List<PhotonPipelineResult> results = sim.getCamera().getAllUnreadResults();
    List<PoseObservation> obs = new LinkedList<PoseObservation>();

    for (int i = results.size() - 1; i >= 0; i--) {

      photonPoseEstimator.addHeadingData(getHeadingData.get().r, getHeadingData.get().t);

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
        // inputs.timestamps = Arrays.copyOf(inputs.timestamps, inputs.timestamps.length + 1);
        // inputs.timestamps[inputs.timestamps.length-1] = results.get(i).getTimestampSeconds();
      } else {
        List<PhotonTrackedTarget> targets = results.get(i).getTargets();
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(results.get(i));

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
    }
    inputs.poseObservations = new PoseObservation[obs.size()];

    for (int b = 0; b <= obs.size() - 1; b++) {
      inputs.poseObservations[b] = obs.get(b);
    }
  }

  @Override
  public void applyCameraTransformation(Transform3d transformation) {
    // visionSystemSim = new VisionSystemSim(sim, transformation);
    // photonPoseEstimator.setRobotToCameraTransform(transformation);
  }

  @Override
  public void setStdDev(Matrix<N3, N1> stddev) {
    this.stddev = stddev;
  }
}
