package frc.robot.subsystems;

import static frc.robot.settings.Constants.Vision.*;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.helpers.MahiDaMath;
import frc.robot.settings.LimelightDetectorData;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Limelight {

  private static Limelight limelight;

  private static Field2d fieldB = new Field2d();
  private static Field2d fieldC = new Field2d();
  private static Field2d field3 = new Field2d();

  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    try {
      FIELD_LAYOUT =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
      FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private Limelight() {
    SmartDashboard.putBoolean("Vision/Left/valid", false);
    SmartDashboard.putBoolean("Vision/Left/trusted", false);
    SmartDashboard.putBoolean("Vision/Right/valid", false);
    SmartDashboard.putBoolean("Vision/Right/trusted", false);
    SmartDashboard.putBoolean("Vision/Extra/valid", false);
    SmartDashboard.putBoolean("Vision/Extra/trusted", false);
    SmartDashboard.putData("Vision/Left/pose", fieldB);
    SmartDashboard.putData("Vision/Right/pose", fieldC);
    SmartDashboard.putData("Vision/Extra/pose", field3);

    // logs active selected path
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public static Limelight getInstance() {
    if (limelight == null) {
      limelight = new Limelight();
    }
    return limelight;
  }

  /**
   * Gets the most recent limelight pose estimate, given that a trustworthy estimate is available.
   * Uses the provided odometryPose for additional filtering.
   *
   * <p>Trusted poses must:
   *
   * <ul>
   *   <li>Be within field bounds.
   *   <li>Have an average tag distance within [MAX_TAG_DISTANCE] from the robot.
   *   <li>Be within [ALLOWABLE_POSE_DIFFERENCE] from the given odometryPose.
   * </ul>
   *
   * @param odometryPose The current odometry pose estimate
   * @return A valid and trustworthy pose. Null if no valid pose. Poses are prioritized by lowest
   *     tagDistance.
   */
  public PoseEstimate getTrustedPose() {
    PoseEstimate poseA =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHTA_NAME);
    PoseEstimate poseB =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHTB_NAME);
    PoseEstimate poseC =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHTC_NAME);
    // we aren't using isTrustworthy here becuase as LL readings have gotten more reliable, we care
    // less about tag distance
    Boolean poseATrust = false;
    Boolean poseBTrust = false;
    Boolean poseCTrust = false;
    if (isConnected(APRILTAG_LIMELIGHTA_NAME)
        && getClosestTagDist(APRILTAG_LIMELIGHTA_NAME) < MAX_TAG_DISTANCE) {
      poseATrust = isValid(APRILTAG_LIMELIGHTA_NAME, poseA);
    }
    if (isConnected(APRILTAG_LIMELIGHTB_NAME)
        && getClosestTagDist(APRILTAG_LIMELIGHTA_NAME) < MAX_TAG_DISTANCE) {
      poseBTrust = isValid(APRILTAG_LIMELIGHTB_NAME, poseB);
    }
    if (isConnected(APRILTAG_LIMELIGHTC_NAME)
        && getClosestTagDist(APRILTAG_LIMELIGHTA_NAME) < MAX_TAG_DISTANCE) {
      poseCTrust = isValid(APRILTAG_LIMELIGHTC_NAME, poseC);
    }
    // if the limelight positions will be merged, let SmartDashboard know!
    boolean mergingPoses = false;
    if (poseATrust && poseBTrust || poseBTrust && poseCTrust || poseATrust && poseCTrust) {
      mergingPoses = true;
    }
    SmartDashboard.putBoolean("LL poses merged", mergingPoses);
    List<String> limelightNames = new ArrayList<>();
    if (poseATrust) {
      limelightNames.add(APRILTAG_LIMELIGHTA_NAME);
    }
    if (poseBTrust) {
      limelightNames.add(APRILTAG_LIMELIGHTB_NAME);
    }
    if (poseCTrust) {
      limelightNames.add(APRILTAG_LIMELIGHTC_NAME);
    }
    if (limelightNames.size() == 0) {
      return null;
    } else if (limelightNames.size() == 1) {
      return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames.get(0));
    } else {
      return mergedPose(limelightNames);
    }
  }
  /**
   * returns a new limelight pose that has the gyroscope rotation of pose1, with the FOMs used to
   * calculate a new pose that proportionally averages the two given positions
   *
   * @param pose1
   * @param pose2
   * @param LL1FOM
   * @param LL2FOM
   * @return
   */
  public PoseEstimate mergedPose(List<String> limelightNames) {
    PoseEstimate pose1 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames.get(0));
    ;
    PoseEstimate pose2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames.get(1));
    ;
    double LL1FOM = getLLFOM(limelightNames.get(0));
    double LL2FOM = getLLFOM(limelightNames.get(1));
    double confidenceSource1 = 1 / Math.pow(LL1FOM, 2);
    double confidenceSource2 = 1 / Math.pow(LL2FOM, 2);
    Pose2d scaledPose1 = MahiDaMath.multiplyOnlyPos(pose1.pose, confidenceSource1);
    Pose2d scaledPose2 = MahiDaMath.multiplyOnlyPos(pose2.pose, confidenceSource2);

    if (limelightNames.size() == 2) {
      Pose2d newPose =
          MahiDaMath.divideOnlyPos(
              (MahiDaMath.addOnlyPosTogether(scaledPose1, scaledPose2)),
              (confidenceSource1 + confidenceSource2));
      pose1.pose = newPose;
    }

    if (limelightNames.size() == 3) {
      PoseEstimate pose3 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames.get(2));
      double LL3FOM = getLLFOM(limelightNames.get(2));
      double confidenceSource3 = 1 / Math.pow(LL3FOM, 2);
      Pose2d scaledPose3 = MahiDaMath.multiplyOnlyPos(pose3.pose, confidenceSource3);
      Pose2d newPose =
          MahiDaMath.divideOnlyPos(
              MahiDaMath.addOnlyPosTogether(
                  MahiDaMath.addOnlyPosTogether(scaledPose1, scaledPose2), scaledPose3),
              (confidenceSource1 + confidenceSource2 + confidenceSource3));
      pose1.pose = newPose;
    }

    return pose1;
  }
  /**
   * calculates the distance to the closest tag that is seen by the limelight
   *
   * @param limelightName
   * @return
   */
  public double getClosestTagDist(String limelightName) {
    double limelight1y = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getY();
    double limelight1x = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getX();
    double limelight1z = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getZ();
    // use those coordinates to find the distance to the closest apriltag for each limelight
    double distance1 = MahiDaMath.DistanceFromOrigin3d(limelight1x, limelight1y, limelight1z);
    return distance1;
  }

  public void updateLoggingWithPoses() {
    if (isConnected(APRILTAG_LIMELIGHTB_NAME)) {
      Pose2d poseB =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHTB_NAME).pose;
      fieldB.setRobotPose(poseB);
      Logger.recordOutput("LeftPose", poseB);
    }

    if (isConnected(APRILTAG_LIMELIGHTC_NAME)) {
      Pose2d poseC =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHTC_NAME).pose;
      fieldC.setRobotPose(poseC);
      Logger.recordOutput("RightPose", poseC);
    }

    if (isConnected(APRILTAG_LIMELIGHTA_NAME)) {
      Pose2d poseA =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(APRILTAG_LIMELIGHTA_NAME).pose;
      fieldC.setRobotPose(poseA);
      Logger.recordOutput("ExtraPose", poseA);
    }

    Pose3d[] AprilTagPoses;

    for (int i = 0; i < 6; i++) {
      if (FIELD_LAYOUT.getTagPose(i) == null) {
        return;
      }

      //  AprilTagPoses[i] = FIELD_LAYOUT.getTagPose((int)
      // LimelightHelpers.getLimelightNTTableEntry("botpose",
      // APRILTAG_LIMELIGHT2_NAME).getInteger(i));
    }

    //  Logger.recordOutput("AprilTagVision", );
    Logger.recordOutput(
        "Vision/targetposes/LeftPose/CameraSpace",
        LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHTB_NAME));
    Logger.recordOutput(
        "Vision/targetposes/RightPose/CameraSpace",
        LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHTC_NAME));
    Logger.recordOutput(
        "Vision/targetposes/ExtraPose/CameraSpace",
        LimelightHelpers.getTargetPose3d_CameraSpace(APRILTAG_LIMELIGHTA_NAME));
    Logger.recordOutput(
        "Vision/targetposes/LeftPose/RobotSpace",
        LimelightHelpers.getTargetPose3d_RobotSpace(APRILTAG_LIMELIGHTB_NAME));
    Logger.recordOutput(
        "Vision/targetposes/LeftPose/RobotSpace",
        LimelightHelpers.getTargetPose3d_RobotSpace(APRILTAG_LIMELIGHTC_NAME));
    Logger.recordOutput(
        "Vision/targetposes/ExtraPose/RobotSpace",
        LimelightHelpers.getTargetPose3d_RobotSpace(APRILTAG_LIMELIGHTA_NAME));
  }

  /**
   * larger FOM is bad, and should be used to indicate that this limelight is less trestworthy
   *
   * @param limelightName
   * @return
   */
  public double getLLFOM(String limelightName) // larger fom is BAD, and is less trustworthy.
      {
    // the value we place on each variable in the FOM. Higher value means it will get weighted more
    // in the final FOM
    /*These values should be tuned based on how heavily you want a contributer to be favored. Right now, we want the # of tags to be the most important
     * with the distance from the tags also being immportant. and the tx and ty should only factor in a little bit, so they have the smallest number. Test this by making sure the two
     * limelights give very different robot positions, and see where it decides to put the real robot pose.
     */
    double distValue = 6;
    double tagCountValue = 7;
    double xyValue = 1;

    // numTagsContributer is better when smaller, and is based off of how many april tags the
    // Limelight identifies
    double numTagsContributer;
    if (limelight.getLLTagCount(limelightName) <= 0) {
      numTagsContributer = 0;
    } else {
      numTagsContributer = 1 / limelight.getLLTagCount(limelightName);
    }
    // tx and ty contributers are based off where on the limelights screen the april tag is. Closer
    // to the center means the contributer will bea smaller number, which is better.
    double centeredTxContributer =
        Math.abs((limelight.getAprilValues(limelightName).tx))
            / 29.8; // tx gets up to 29.8, the closer to 0 tx is, the closer to the center it is.
    double centeredTyContributer =
        Math.abs((limelight.getAprilValues(limelightName).ty))
            / 20.5; // ty gets up to 20.5 for LL2's and down. LL3's go to 24.85. The closer to 0 ty
    // is, the closer to the center it is.
    // the distance contributer gets smaller when the distance is closer, and is based off of how
    // far away the closest tag is
    double distanceContributer = (limelight.getClosestTagDist(limelightName) / 5);

    // calculates the final FOM by taking the contributors and multiplying them by their values,
    // adding them all together and then dividing by the sum of the values.
    double LLFOM =
        ((distValue * distanceContributer)
                    + (tagCountValue * numTagsContributer)
                    + (centeredTxContributer * xyValue)
                    + (centeredTyContributer))
                / distValue
            + tagCountValue
            + xyValue
            + xyValue;
    Logger.recordOutput("Vision/LLFOM" + limelightName, LLFOM);
    return LLFOM;
  }

  public LimelightDetectorData getNeuralDetectorValues() {
    return new LimelightDetectorData(
        LimelightHelpers.getTX(APRILTAG_LIMELIGHTC_NAME),
        LimelightHelpers.getTY(APRILTAG_LIMELIGHTC_NAME),
        LimelightHelpers.getTA(APRILTAG_LIMELIGHTC_NAME),
        LimelightHelpers.getNeuralClassID(APRILTAG_LIMELIGHTC_NAME),
        LimelightHelpers.getTV(APRILTAG_LIMELIGHTC_NAME));
  }

  public LimelightDetectorData getAprilValues(String cameraname) {
    return new LimelightDetectorData(
        LimelightHelpers.getTX(cameraname),
        LimelightHelpers.getTY(cameraname),
        LimelightHelpers.getTA(cameraname),
        LimelightHelpers.getNeuralClassID(cameraname),
        LimelightHelpers.getTV(cameraname));
  }

  public int getLLTagCount(String cameraname) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraname).tagCount;
  }

  private boolean isValid(String limelightName, PoseEstimate estimate) {
    Boolean valid =
        (estimate.pose.getX() < FIELD_CORNER.getX()
            && estimate.pose.getX() > 0.0
            && estimate.pose.getY() < FIELD_CORNER.getY()
            && estimate.pose.getY() > 0.0);

    if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHTB_NAME)) {
      SmartDashboard.putBoolean("Vision/Left/valid", valid);
      SmartDashboard.putNumber("Vision/Left/Stats/valid", (valid ? 1 : 0));
      SmartDashboard.putNumber("Vision/Left/Stats/avgTagDist", estimate.avgTagDist);
      SmartDashboard.putNumber("Vision/Left/Stats/tagCount", estimate.tagCount);
      SmartDashboard.putNumber("Vision/Left/Stats/latency", estimate.latency);
    } else if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHTC_NAME)) {
      SmartDashboard.putBoolean("Vision/Right/valid", valid);
      SmartDashboard.putNumber("Vision/Right/Stats/valid", (valid ? 1 : 0));
      SmartDashboard.putNumber("Vision/Right/Stats/avgTagDist", estimate.avgTagDist);
      SmartDashboard.putNumber("Vision/Right/Stats/tagCount", estimate.tagCount);
      SmartDashboard.putNumber("Vision/Right/Stats/latency", estimate.latency);
    } else if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHTA_NAME)) {
      SmartDashboard.putBoolean("Vision/Extra/valid", valid);
      SmartDashboard.putNumber("Vision/Extra/Stats/valid", (valid ? 1 : 0));
      SmartDashboard.putNumber("Vision/Extra/Stats/avgTagDist", estimate.avgTagDist);
      SmartDashboard.putNumber("Vision/Extra/Stats/tagCount", estimate.tagCount);
      SmartDashboard.putNumber("Vision/Extra/Stats/latency", estimate.latency);
    } else {
      System.err.println("Limelight name is invalid. (limelight.isValid)");
    }
    return valid;
  }
  /**
   * checks if the robotPose returned by the limelight is within the field and stable. It does this
   * by running isValid() with the limelight, and checking if the limelight's pose either contains
   * 2+ tags or is closer then MAX_TAG_DISTANCE (from constants) from the tag.
   *
   * @param limelightName the name of the requested limelight, as seen on NetworkTables
   * @param estimate the poseEstimate from that limelight
   * @param odometryPose the robot's pose from the DriveTrain, unused right now
   * @return true if the pose is within the field bounds and the tag distance is less than 7
   */
  private boolean isTrustworthy(String limelightName, PoseEstimate estimate, Pose2d odometryPose) {
    Boolean trusted = (isValid(limelightName, estimate) && estimate.avgTagDist < 7);

    if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHTB_NAME)) {
      SmartDashboard.putBoolean("Vision/Left/trusted", trusted);
      SmartDashboard.putNumber("Vision/Left/Stats/trusted", (trusted ? 1 : 0));
    } else if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHTC_NAME)) {
      SmartDashboard.putBoolean("Vision/Right/trusted", trusted);
      SmartDashboard.putNumber("Vision/Right/Stats/trusted", (trusted ? 1 : 0));
    } else if (limelightName.equalsIgnoreCase(APRILTAG_LIMELIGHTA_NAME)) {
      SmartDashboard.putBoolean("Vision/Extra/trusted", trusted);
      SmartDashboard.putNumber("Vision/Extra/Stats/trusted", (trusted ? 1 : 0));
    } else {
      System.err.println("Limelight name is invalid. (limelight.isTrustworthy)");
    }
    return trusted;
  }
  /**
   * @param limelightName
   * @return a boolean of wether or not the data from the limelight is accessible. If false,
   *     limelight may not be connected
   */
  public boolean isConnected(String limelightName) {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    double tx = limelightTable.getEntry("tx").getDouble(Double.NaN);
    boolean connected = !Double.isNaN(tx);
    return connected;
  }
}
