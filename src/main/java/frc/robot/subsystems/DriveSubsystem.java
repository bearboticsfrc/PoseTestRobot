// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.RateLimiter;
import java.util.Map;

/** Controls the four swerve modules for autonomous and teleoperated modes. */
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule frontLeft;
  private final SwerveModule backLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backRight;

  // The gyro sensor
  private final WPI_Pigeon2 gyro;
  private double gyroOffsetDegrees = 0.0;

  private double kMaxSpeed = 3.5; // DriveConstants.kMaxSpeedMetersPerSecond;

  private double speed = 1.4; // kMaxSpeed / 2;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry;

  ShuffleboardTab driveSystemTab = Shuffleboard.getTab("Drive System");
  ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

  GenericEntry maxSpeedEntry =
      compTab
          .add("Drive Speed", speed)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withSize(2, 1)
          .withPosition(3, 1)
          .withProperties(Map.of("min", 0, "max", kMaxSpeed))
          .getEntry();

  private boolean turboMode = false;
  private boolean fieldRelativeMode = true;

  private final RateLimiter xLimiter =
      new RateLimiter(
          DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
          DriveConstants.kTeleDriveMaxDecelerationUnitsPerSecond);
  private final RateLimiter yLimiter =
      new RateLimiter(
          DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
          DriveConstants.kTeleDriveMaxDecelerationUnitsPerSecond);
  private final RateLimiter turningLimiter =
      new RateLimiter(
          DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond,
          DriveConstants.kTeleDriveMaxAngularDecelerationUnitsPerSecond);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro = new WPI_Pigeon2(DriveConstants.kPigeon2_CANid);

    compTab.addNumber("gyro", this::getHeading).withPosition(5, 1);

    // if (gyro.getState() != PigeonState.Ready) {
    // DriverStation.reportWarning("Gyro not ready!", false);
    //  while (gyro.getState() != PigeonState.Ready );
    //  DriverStation.reportWarning("Gyro ready", false);
    // }
    zeroHeading();

    frontLeft =
        new SwerveModule(
            "FL",
            driveSystemTab,
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftTurningInputPort,
            DriveConstants.kFrontLeftEncoderAngle);
            
    backLeft =
        new SwerveModule(
            "BL",
            driveSystemTab,
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftTurningInputPort,
            DriveConstants.kBackLeftEncoderAngle);

    frontRight =
        new SwerveModule(
            "FR",
            driveSystemTab,
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightTurningInputPort,
            DriveConstants.kFrontRightEncoderAngle);

    backRight =
        new SwerveModule(
            "BR",
            driveSystemTab,
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightTurningInputPort,
            DriveConstants.kBackRightEncoderAngle);

    odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions());

  }

  public void setTurboMode(boolean mode) {
    if (mode) {
      turboMode = true;
      speed *= 2;
      speed = Math.min(speed * 2.0, kMaxSpeed);
    } else {
      turboMode = false;
      speed /= 2;
    }
    maxSpeedEntry.setDouble(speed);
  }

  public boolean getTurboMode() {
    return turboMode;
  }

  public void setFieldRelative(boolean mode) {
    fieldRelativeMode = mode;
  }

  public boolean getFieldRelative() {
    return fieldRelativeMode;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        gyro.getRotation2d(),
        getModulePositions());
    speed = maxSpeedEntry.getDouble(kMaxSpeed);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

 // public Vector2d getVelocity() {
 //   return new Vector2d(0.0, 0.0);
 // }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(),getModulePositions(),pose);
  }

  public SwerveModulePosition [] getModulePositions() {
    return new SwerveModulePosition [] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }
  public SwerveModuleState [] getModuleStates() {
    return new SwerveModuleState [] {
      frontLeft.getState(),
      backLeft.getState(),
      frontRight.getState(),
      backRight.getState()
    };
  }

  /**
   * Resets the odometry to the specified pose of a state in a PathPlanner trajectory.
   *
   * @param state The state of the PathPlanner trajectory to contstruct a pose.
   */
  public void resetOdometry(PathPlannerState state) {
    resetOdometry(new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
  }

  /**
   * Drive robot using Joystick inputs, default to CENTER pivot, and sets field relative to the
   * current fieldRelativeMode setting.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    this.drive(xSpeed, ySpeed, rot, fieldRelativeMode);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    rot,
                    gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));

    setModuleStates(swerveModuleStates);
  }

  public void stop() {
    drive(0.0, 0.0, 0.0);
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);
    frontLeft.set(
        swerveModuleStates[0].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE,
        swerveModuleStates[0].angle.getRadians());
    frontRight.set(
        swerveModuleStates[1].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE,
        swerveModuleStates[1].angle.getRadians());
    backLeft.set(
        swerveModuleStates[2].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE,
        swerveModuleStates[2].angle.getRadians());
    backRight.set(
        swerveModuleStates[3].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE,
        swerveModuleStates[3].angle.getRadians());
  }
  /**
   * Sets the swerve ModuleStates during auto.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStatesForAuto(SwerveModuleState[] swerveModuleStates) {
    double fudgeFactor = 1.0; // was 1.1
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.set(
        swerveModuleStates[0].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE
            * fudgeFactor,
        swerveModuleStates[0].angle.getRadians());
    frontRight.set(
        swerveModuleStates[1].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE
            * fudgeFactor,
        swerveModuleStates[1].angle.getRadians());
    backLeft.set(
        swerveModuleStates[2].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE
            * fudgeFactor,
        swerveModuleStates[2].angle.getRadians());
    backRight.set(
        swerveModuleStates[3].speedMetersPerSecond
            / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE
            * fudgeFactor,
        swerveModuleStates[3].angle.getRadians());
  }

  /** Sets the swerve modules to cross each other. */
  public void setXMode() {
    frontLeft.set(0.0, Math.toRadians(DriveConstants.kXModeAngle));
    frontRight.set(0.0, Math.toRadians(DriveConstants.kXModeAngle));
    backLeft.set(0.0, Math.toRadians(DriveConstants.kXModeAngle));
    backRight.set(0.0, Math.toRadians(DriveConstants.kXModeAngle));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    //   frontLeft.resetEncoders();
    //   backLeft.resetEncoders();
    //   frontRight.resetEncoders();
    //   backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    headingOffset(0);
    gyro.reset();
    gyro.addYaw(0);
  }

  private void headingOffset(double offsetDegrees) {
    System.out.println("!!!!!!!!!!! Adding " + offsetDegrees);
    gyroOffsetDegrees = offsetDegrees;
    gyro.addYaw(gyroOffsetDegrees);
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    double temp = gyro.getRotation2d().getDegrees();
    temp -= Math.floor(temp / 360.0) * 360.0;
    return temp;
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate();
  }
}
