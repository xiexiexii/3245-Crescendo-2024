// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.VisionDataProvider;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private MAXSwerveModule[] modules = new MAXSwerveModule[]{
    m_frontLeft,
    m_frontRight,
    m_rearLeft,
    m_rearRight
  };


  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  private final VisionDataProvider m_visionDataProvider = new VisionDataProvider(VisionConstants.kCameraName);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private Rotation2d rawGyroRotation = new Rotation2d();
  public static final PIDConstants translationalPID = new PIDConstants(0.824, 0.95, 0.15);
  public static final PIDConstants rotationalPID = new PIDConstants(0.23, 0, 0);

  public static final HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(translationalPID, rotationalPID,
    5.7, DriveConstants.kWheelBase/Math.sqrt(2), new ReplanningConfig());
  
  // Pose estimator class for tracking robot pose
  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());
  
  private Alliance m_alliance = Alliance.Blue;
  private boolean m_usingVision = true;

  private Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::setPose, 
      () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()), 
      this::runVelocity, config, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
        this);

    m_visionDataProvider.setFiducialAreaThreshold(VisionConstants.kFiducialAreaThreshold);
    m_visionDataProvider.setFiducialDistanceThreshold(VisionConstants.kFiducialDistanceThreshold);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        Rotation2d.fromDegrees(-m_gyro.getYaw()),
        //TODO: make above negative
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    if (m_usingVision) {
      Optional<LimelightHelpers.PoseEstimate> estimate;
      if (DriverStation.isDisabled()) {
        estimate = m_visionDataProvider.getEstimatedGlobalPoseWithoutFiltering(getPose());
      } else {
        estimate = m_visionDataProvider.getEstimatedGlobalPose(getPose());
      }
      estimate.ifPresent((e) -> {
        SmartDashboard.putString("Robot pose (2D)", e.pose.toString());
        m_poseEstimator.addVisionMeasurement(e.pose, e.timestampSeconds);
      });
    }
    SmartDashboard.putNumber("NavX yaw", -m_gyro.getYaw());
    SmartDashboard.putNumber("NavX cumulative angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Actual heading", getHeading().getDegrees());

    rawGyroRotation = m_gyro.getRotation2d();
    m_field.setRobotPose(getPose());
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Resets the pose estimator to the specified pose. Because this method will forcibly modify a pose to a set
   * position, you should not call it while the robot is in motion.
   *
   * @param pose The pose to which to set the estimator.
   */
  public void resetEstimator(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Gets the provider for vision data on this robot.
   * @return The {@link frc.robot.utils.VisionDataProvider} that this drivetrain uses.
   */
  public VisionDataProvider getVisionDataProvider() {
    return m_visionDataProvider;
  }

  /**
   * Returns true if the pose estimator is using vision to find a global pose.
   */
  public boolean isUsingVision() {
    return m_usingVision;
  }

  /**
   * Set whether the pose estimator uses vision to find a global pose.
   */
  public void setUsingVision(boolean usingVision) {
    m_usingVision = usingVision;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;
    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        //take shortest path possible to desired pose
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
          //wraps angle to lie within 2pi
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds;
    if (fieldRelative) {
      Rotation2d heading;
      // These are switched because the drive command is completely reversed
      switch (DriverStation.getAlliance().orElse(m_alliance)) {
        case Red:
          heading = getHeading();
          break;
        case Blue:
        default:
          heading = getHeading().rotateBy(Rotation2d.fromDegrees(180));
          break;
      }
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, heading);
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    }

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

    public Command getAuto(String autoName) {
      return AutoBuilder.buildAuto(autoName);
    }

    public Command getPath(String pathName) {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] currentStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++){
      currentStates[i] = modules[i].getState();
    }
    return currentStates;
      
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i <4; i++){
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void setChassisSpeeds (ChassisSpeeds desiredSpeeds){
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(desiredSpeeds));
  }

  public void runVelocity(ChassisSpeeds speeds) {
    //calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);

    //send setpoints to modules
    for (int i = 0; i < 4; i++) {
      //the module returns the optimizes state, useful for logging
      modules[i].setDesiredState(setpointStates[i]);
    }
  }

  public ChassisSpeeds getChassisSpeeds(){
    /*return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()); 
      */
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }


  
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
  public void calibrateGyro(){
    //m_gyro.calibrate();
  }

  public void rumble(CommandXboxController controller){
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a {@link edu.wpi.first.math.geometry.Rotation2d}
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
