// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class ControllerConstants {
    //OI
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.075;

    //driver controls
    public static final int kVisionTurnButton = 4; //Y
    public static final int intakeSeqAxis = 3; //right trig
    public static final int spinupButton = 6; //right bump
    public static final int shootButton = 5; //left bump

    public static final int indexShootButton = 6; //Double check, we want right bumper
    public static final int indexShootStopButton = 5; // double check, we want left bumper
    public static final int intakeAxis = 3; //right trigger
    public static final int intakeFlipoutAxis = 2; //left trigger

    //manual controls for testing, operator
    public static final int intakeInButton = 6; //right bump
    public static final int intakeOutButton = 5; //left bump
    public static final int intakeTopAxis = 3;
    public static final int intakeBottomAxis = 2;
    
    public static final int flipOutManualAxis = 1; //left joy
    public static final int flipOutRunButton = 1; //A

    public static final int indexerButton = 5;

    public static final int shoulderAxis = 1; //left joy

    public static final int indexerMaxButton = 1; //A
    public static final int indexerMinButton = 2; //B
    public static final int indexer0Button = 3; //X
    public static final int indexer45Button = 4; //Y
    public static final int indexer180Button = 6; //right bumper

    //public static final int shootButton = 6; //right bumper
    public static final int shootBackButton = 2; //B

    public static final int climbAxis = 2; //also unsure lmao

    public static final int shoulderHomeButton = 2; //B
    public static final int shoulderAmpButton = 1; //A
    public static final int shoulderProtButton = 3; //X

    public static final int bumpUpButton = 1; //A
    public static final int bumpDownButton = 2; //B
  }

  public static class PIDConstants {
    public static double shoulderkF = 0;
    public static double shoulderkP = 0.95;
    public static double shoulderkI = 0;
    public static double shoulderkD = 0.0;

    public static double flipoutkF = 0;
    public static double flipoutkP = 0.1;
    public static double flipoutkI = 0;
    public static double flipoutkD = 0.05;

    public static final double kDefaultAutomaticTurningP = 0.02;
    public static final double kDefaultAutomaticTurningI = 0;
    public static final double kDefaultAutomaticTurningD = 0.00145;
  }

  public static class MotorIDConstants {
    // public static final int intakeBottomMotorID = 31;
    public static final int intakeTop1MotorID = 32;
    public static final int intakeTop2MotorID = 33;
    public static final int intakeFlipoutMotorID = 34;
    public static final int intakeRunExtended = 35;

    public static final int shoulder1MotorID = 13;
    public static final int shoulder2MotorID = 12;
    public static final int shoulder3MotorID = 11;
    public static final int shoulder4MotorID = 14;

    public static final int indexerMotorID = 23;
    public static final int shootMotor1ID = 21;
    public static final int shootMotor2ID = 22;
    public static final int climbMotorID = 50;

    public static final int blinkinPort = 9;

    public static final int shoulderCANCoderID = 15;
  }

  public static class SensorConstants {
    public static final int hallEffectDIOPort = 7;
    public static final int intakeBeamBreakDIOPort = 8;
    public static final int intakeBeamBreakDIOOutput = 9;
    public static final int indexBeamBreakDIOPort = 3;
    public static final int colorSensorPort = 8;
  }

  public static class MotorSpeedsConstants {
    public static final double intakeNeoSpeed = 0.95;
    public static final double intakeFalconSpeed = 0.4;
    public static final double intakeNeoFeedSpeed = 0.2;
    public static final double intakeFalconFeedSpeed = 0.2;

    public static final double flipOutClosedMaxSpeed = 1;
    public static final double flipoutOpenMaxSpeed = 0.4;
    public static final double flipOutRamp = 0.15;
    public static final double flipOutRunSpeed = -0.9;

    public static final double shoulderOpenMaxSpeed = 0.2;
    public static final double shoulderClosedMaxSpeed = 0.3; // 0.95
    public static final double shoulderRampRate = 0.4;
    public static final double shoot1MaxVal = -0.75;
    public static final double shoot2MaxVal = .75;
    public static final double shoot1MaxValAmp = -0.3;
    public static final double shoot2MaxValAmp = 0.3;
    public static final double shoot1MaxValAuto = -0.65;
    public static final double shoot2MaxValAuto = 0.65;
    public static final double shoot1RunBackVal = -0.13;
    public static final double shoot2RunBackVal = 0.13;

    public static final double indexRunSuckIn = 0.5;
    public static final double indexRunMoveBack = 0.90;
    public static final double climbMaxVal = 0.7;
  }
  
  public static class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    
    public static final double kMaxSpeedMetersPerSecond = 5.5; //max is 5.74 W/ votrexes -Neeks
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 4*Math.PI; // radians per second
    public static final double kMagnitudeSlewRate = 8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 6; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 3;

    public static final boolean kGyroReversed = true;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.74;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.08;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 25; // amps
  }

  public static class PositionValueConstants {
    public static final double shoulderHomePos = 0;
    public static final double shoulderAmpShotPos = 0.65; //perfect DO NOT CHANGE
    public static final double shoulderProtShotPos = 0.2;
    public static final double minShoulderPos = 0;
    public static final double maxShoulderPos = 0.5;
    public static final double minDistanceFromSpeaker = 1.252;

    public static final double flipoutOutPos = 15.5;
    public static final double flipoutInPos = 2;
  }
  
  public static class TeleopTimeConstants {
    public static final double indexerShootSpinAfterTime = 0.1;
    public static final double handoffIndexerSlowTime = 0.1;
    public static final double handoffIndexerReverseTime = 0.05;
    public static final double autoHandoffIndexerReverseTime = 0.175;
  }

  public static class AutoTimeConstants {
    public static final double spinUpAutoTime1 = 0.65;
    public static final double spinUpAutoTime2 = 0.6;
    public static final double indexAutoTime2 = 0.2;
    public static final double indexAutoTime1 = spinUpAutoTime1 + 0.2;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "limelight";
    public static final Transform3d kRobotToCameraTransform = new Transform3d(Units.inchesToMeters(11.969),
                                                                              Units.inchesToMeters( 5.000),
                                                                              Units.inchesToMeters( 9.321),
                                                                              new Rotation3d(0,
                                                                                             Units.degreesToRadians(-30),
                                                                                             0));
                                                                                    
    public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final double kFiducialAreaThreshold = 0.02;
    public static final double kFiducialDistanceThreshold = 4;
  }

  public static final class PreferenceKeys {
    public static final String kAutomaticTurningP = "VisionTurningP";
    public static final String kAutomaticTurningI = "VisionTurningI";
    public static final String kAutomaticTurningD = "VisionTurningD";
  }

}
