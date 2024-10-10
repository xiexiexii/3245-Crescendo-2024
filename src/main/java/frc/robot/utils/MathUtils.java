// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public final class MathUtils {

  public static double getYawToPose(Pose2d robotPose, Pose2d targetPose) {
    // From PhotonLib
    Translation2d relativeTranslation = targetPose.relativeTo(robotPose).getTranslation();
    return new Rotation2d(relativeTranslation.getX(), relativeTranslation.getY()).getDegrees();
  }

  private MathUtils() {}
}
