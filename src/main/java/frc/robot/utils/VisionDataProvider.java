// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.LimelightHelpers.LimelightResults;

/** A VisionDataProvider provides vision data from a Limelight camera. */
public class VisionDataProvider {

  private final String m_limelightName;

  private double m_averageTagAreaThreshold = 0;
  private double m_averageTagDistanceThresholdMeters = Double.POSITIVE_INFINITY;
  private boolean m_usingFiltering = true;
  
  /**
   * Creates a new VisionDataProvider.
   * 
   * @param cameraName The name of the camera.
   * @param robotToCameraTransform The 3D transform from the robot's center to the camera.
   * @param field The AprilTag field layout.
   */
  public VisionDataProvider(String cameraName) {
    m_limelightName = cameraName;
  }

  /**
   * Gets the name of the Limelight providing the vision data.
   * @return The name of the Limelight.
   */
  public String getCameraName() {
    return m_limelightName;
  }

  /**
   * Gets the targets the camera currently sees. It is stronly recommended that you call this method only once per
   * update to guarantee the fetched targets are from the same vision snapshot.
   * @return A list of the targets the camera currently sees.
   */
  public List<LimelightHelpers.LimelightTarget_Fiducial> getTargets() {
    LimelightResults result = LimelightHelpers.getLatestResults(m_limelightName);
    return Arrays.asList(result.targetingResults.targets_Fiducials);
  }

  /**
   * Get the minimum average area-of-image-consumed for each of the reference fiducials before the pose is rejected.
   * @return A value between 0 and 1, which is the area threshold.
   */
  public double getFiducialAreaThreshold() {
    return m_averageTagAreaThreshold;
  }

  /**
   * Set the minimum average area-of-image-consumed for each of the reference fiducials before the pose is rejected.
   * @param fiducialAreaThreshold A value between 0 and 1, which is the area threshold.
   */
  public void setFiducialAreaThreshold(double fiducialAreaThreshold) {
    if (fiducialAreaThreshold < 0 || fiducialAreaThreshold > 1) {
      throw new IllegalArgumentException("Fiducial area threshold must be between 0 and 1");
    }
    m_averageTagAreaThreshold = fiducialAreaThreshold;
  }

  /**
   * Get the maximum average distance each of the reference fiducials before the pose is rejected.
   * @return The maximum average distance to each of the reference fiducials in meters.
   */
  public double getFiducialDistanceThreshold() {
    return m_averageTagDistanceThresholdMeters;
  }

  /**
   * Set the maximum average distance each of the reference fiducials before the pose is rejected.
   * @param fiducialAreaThreshold The maximum average distance to each of the reference fiducials in meters.
   */
  public void setFiducialDistanceThreshold(double fiducialDistanceThreshold) {
    if (fiducialDistanceThreshold < 0) {
      throw new IllegalArgumentException("Fiducial distance threshold must be non-negative");
    }
    m_averageTagDistanceThresholdMeters = fiducialDistanceThreshold;
  }

  /**
   * Return true if the vision data provider is filtering the robot pose; false otherwise.
   */
  public boolean usingFiltering() {
    return m_usingFiltering;
  }

  /**
   * Set whether the vision data provider should filter robot poses.
   * @param useFiltering Whether the vision data provider should filter robot poses.
   */
  public void setUseFiltering(boolean useFiltering) {
    m_usingFiltering = useFiltering;
  }

  /**
   * Get the estimated global pose of the robot using the vision data, if it exists, using the previous pose to help
   * refine the estimate. If the vision system can't estimate a robot pose when this is called or filtering is enabled
   * and the filter conditions are not met, the result will be empty.
   * @param previousEstimatedRobotPose The previously-estimated robot pose.
   * @return The newly estimated robot pose if a good estimate could be made, otherwise Optional.empty().
   */
  public Optional<LimelightHelpers.PoseEstimate> getEstimatedGlobalPose(Pose2d previousEstimatedRobotPose) {
    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
    // Limelight's system returns the origin instead of null when it can't find a pose. Fortunately the origin is an
    // invalid pose.
    if (estimate.pose.getX() < 0.01
        || m_usingFiltering && (
            estimate.avgTagArea < m_averageTagAreaThreshold
            || estimate.avgTagDist > m_averageTagDistanceThresholdMeters)) {
      return Optional.empty();
    } else {
      return Optional.of(estimate);
    }
  }

  /**
   * Get the estimated global pose of the robot using the vision data, if it exists, using the previous pose to help
   * refine the estimate. If the vision system can't estimate a robot pose when this is called, the result will be empty.
   * @param previousEstimatedRobotPose The previously-estimated robot pose.
   * @return The newly estimated robot pose if a good estimate could be made, otherwise Optional.empty().
   */
  public Optional<LimelightHelpers.PoseEstimate> getEstimatedGlobalPoseWithoutFiltering(Pose2d previousEstimatedRobotPose) {
    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);
    // Limelight's system returns the origin instead of null when it can't find a pose. Fortunately the origin is an
    // invalid pose.
    if (estimate.pose.getX() < 0.01) {
      return Optional.empty();
    } else {
      return Optional.of(estimate);
    }
  }
}
