package team5427.frc.robot.subsystems.vision.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import team5427.frc.robot.subsystems.vision.VisionConstants;
import team5427.lib.detection.CoordinateUtils;

public class VisionIOQuestNav implements VisionIO {

  public QuestNav questNav;
  private Transform3d robotToQuestTransformation;

  public VisionIOQuestNav() {
    questNav = QuestNav.getInstance();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = questNav.getConnected();
    ArrayList<PoseObservation> questPoseObservation = new ArrayList<>();
    questPoseObservation.add(
        new PoseObservation(
            questNav.getTimestamp(),
            questNav.getPose3d().transformBy(robotToQuestTransformation.inverse()),
            0,
            0,
            0,
            0,
            0,
            PoseObservationType.QUEST_NAV));
  }

  public void log() {
    Logger.recordOutput(
        VisionConstants.kQuestLoggingDirectory + "Battery", questNav.getBatteryPercent());
    Logger.recordOutput(
        VisionConstants.kQuestLoggingDirectory + "TrackingStatus", questNav.getTrackingStatus());
    Logger.recordOutput(
        VisionConstants.kQuestLoggingDirectory + "FrameCount", questNav.getFrameCount());
    Logger.recordOutput(
        VisionConstants.kQuestLoggingDirectory + "TrackingLostCounter",
        questNav.getTrackingLostCounter());
  }

  @Override
  public void applyCameraTransformation(Transform3d transformation) {
    this.robotToQuestTransformation = transformation;
  }

  /**
   * @param questPosition the position of the quest itself, NOT the robot
   */
  public void setQuestPosition(Pose2d questPosition) {
    questNav.setPose(questPosition);
  }

  /**
   * @param robotPosition the position of the robot, applies the robotToQuestTransformation to set
   *     quest pose
   */
  public void setRobotPosition(Pose2d robotPosition) {
    questNav.setPose(
        robotPosition.transformBy(CoordinateUtils.transform3dTo2d(robotToQuestTransformation)));
  }
}
