package team5427.frc.robot.subsystems.vision.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import limelight.Limelight;
import limelight.networktables.LimelightResults;

public class VisionIOLimelight implements VisionIO {

  private Limelight limelight;

  public VisionIOLimelight(String cameraName, Transform3d cameraTransform) {
    limelight = new Limelight(cameraName);
    limelight.getSettings().withCameraOffset(Pose3d.kZero.plus(cameraTransform)).save();
    limelight.flush();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    Optional<LimelightResults> results = limelight.getLatestResults();
    if (results.isPresent()) {
      inputs.connected = true; // placeholder
      LimelightResults res = results.get();
      PoseObservation[] observations = new PoseObservation[res.targets_Fiducials.length];
      Set<Short> tagIdSet = new HashSet<Short>();
      for (int i = 0; i < observations.length; i++) {
        tagIdSet.add((short) res.targets_Fiducials[i].fiducialID);
        observations[i] =
            new PoseObservation(
                res.timestamp_LIMELIGHT_publish,
                null,
                -1,
                res.targets_Fiducials.length,
                0,
                res.targets_Fiducials[i].tx,
                res.targets_Fiducials[i].ty,
                PoseObservationType.NONE);
      }
      inputs.tagIds = new int[tagIdSet.size()];
      for (int j = 0; j < tagIdSet.size(); j++) {
        inputs.tagIds[j] = (Short) tagIdSet.toArray()[j];
      }
      inputs.poseObservations = observations;
    }
  }

  @Override
  public void applyCameraTransformation(Transform3d transformation) {
    limelight.getSettings().withCameraOffset(Pose3d.kZero.plus(transformation)).save();
  }
}
