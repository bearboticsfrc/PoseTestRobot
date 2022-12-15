package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.CTREUtil;
import frc.robot.util.RevUtil;

/** A SwerveModules consists of a drive motor and a steer motor */
public class SwerveModule {

  private CANSparkMax driveMotor;
  private RelativeEncoder driveEncoder;
  private CANSparkMax steerMotor;
  private RelativeEncoder steerBuiltinEncoder;
  private SparkMaxPIDController steerMotorController;
  private CANCoder steerAbsoluteEncoder;
  private double referenceAngleRadians = 0;

  public SwerveModule(
      String name,
      ShuffleboardContainer dashboardContainer,
      int driveMotorCanId,
      int steerMotorCanId,
      int steerEncoderCanId,
      double steerAngleOffset) {
    setupDriveMotor(driveMotorCanId);
    setupCANCoder(steerEncoderCanId, steerAngleOffset);
    setupSteerMotor(steerMotorCanId);

    dashboardContainer.addNumber(
        name + "Current Angle", () -> Math.toDegrees(getSteerAngle()));
    dashboardContainer.addNumber(
        name + "Target Angle", () -> Math.toDegrees(getReferenceAngle()));
    dashboardContainer.addNumber(
        name + "Absolute Encoder Angle", () -> Math.toDegrees(getAbsoluteAngle()));
    dashboardContainer.addNumber(name + "Current Velocity", this::getDriveVelocity);
  }

  private void setupDriveMotor(int id) {
    driveMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor.setInverted(true);

    // Setup voltage compensation
    RevUtil.checkRevError(
        driveMotor.enableVoltageCompensation(DriveConstants.kNominalVoltage),
        "Failed to enable voltage compensation");
    RevUtil.checkRevError(
        driveMotor.setSmartCurrentLimit(DriveConstants.kDriveCurrentLimit),
        "Failed to set current limit for NEO");
    RevUtil.checkRevError(
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20),
        "Failed to set periodic status frame 0 rate");
    RevUtil.checkRevError(
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate");
    RevUtil.checkRevError(
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
        "Failed to set periodic status frame 2 rate");
    RevUtil.checkRevError(
        driveMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500),
        "Failed to set periodic status frame 2 rate");
    // Set neutral mode to brake
    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Setup encoder
    driveEncoder = driveMotor.getEncoder();
    double positionConversionFactor =
        Math.PI * DriveConstants.kWheelDiameterMeters * DriveConstants.kDriveGearReduction;
    driveEncoder.setPositionConversionFactor(positionConversionFactor);
    driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
  }

  public void setupSteerMotor(int id) {
    double pidProportional = 1.0;
    double pidIntegral = 0.0;
    double pidDerivative = 0.1;

    steerMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    RevUtil.checkRevError(
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
        "Failed to set periodic status frame 0 rate");
    RevUtil.checkRevError(
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate");
    RevUtil.checkRevError(
        steerMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
        "Failed to set periodic status frame 2 rate");
    RevUtil.checkRevError(steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
    steerMotor.setInverted(false);
    RevUtil.checkRevError(
        steerMotor.enableVoltageCompensation(DriveConstants.kNominalVoltage),
        "Failed to enable voltage compensation");
    RevUtil.checkRevError(
        steerMotor.setSmartCurrentLimit(DriveConstants.kSteerCurrentLimit),
        "Failed to set NEO current limits");

    steerBuiltinEncoder = steerMotor.getEncoder();
    RevUtil.checkRevError(
        steerBuiltinEncoder.setPositionConversionFactor(
            2.0 * Math.PI * DriveConstants.kSteerDriveReduction),
        "Failed to set NEO encoder conversion factor");
    RevUtil.checkRevError(
        steerBuiltinEncoder.setVelocityConversionFactor(
            2.0 * Math.PI * DriveConstants.kSteerDriveReduction / 60.0),
        "Failed to set NEO encoder conversion factor");
    RevUtil.checkRevError(
        steerBuiltinEncoder.setPosition(getAbsoluteAngle()), "Failed to set NEO encoder position");

    steerMotorController = steerMotor.getPIDController();
    RevUtil.checkRevError(steerMotorController.setP(pidProportional), "Failed to set NEO PID proportional constant");
    RevUtil.checkRevError(steerMotorController.setI(pidIntegral), "Failed to set NEO PID integral constant");
    RevUtil.checkRevError(steerMotorController.setD(pidDerivative), "Failed to set NEO PID derivative constant");

    RevUtil.checkRevError(
        steerMotorController.setFeedbackDevice(steerBuiltinEncoder), "Failed to set NEO PID feedback device");
  }

  public void setupCANCoder(int id, double offsetDegrees) {
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = offsetDegrees;
    config.sensorDirection = false;

    steerAbsoluteEncoder = new CANCoder(id);
    CTREUtil.checkCtreError(steerAbsoluteEncoder.configAllSettings(config, 250), "Failed to configure CANCoder");
    CTREUtil.setPeriodicFrameRate(steerAbsoluteEncoder);
  }

  
  public double getAbsoluteAngle() {
    double angle = Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition());
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    CTREUtil.checkCtreError(steerAbsoluteEncoder.getLastError(), "Last CANCoder Error");    

    CANCoderStickyFaults faults = new CANCoderStickyFaults();
    CTREUtil.checkCtreError(steerAbsoluteEncoder.getStickyFaults(faults), " Error getting sticky faults");
    if (faults.hasAnyFault()) {
      DriverStation.reportError(String.format("CANCoder fault: %s", faults.toString()), false);
      steerAbsoluteEncoder.clearStickyFaults();
    }

    return angle;
  }

  public double getReferenceAngle() {
    return referenceAngleRadians;
  }

  private static final int ENCODER_RESET_ITERATIONS = 500;
  private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
  private double resetIteration = 0;

  public void setReferenceAngle(double referenceAngleRadians) {
    double currentAngleRadians = steerBuiltinEncoder.getPosition();

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and
    // we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (steerBuiltinEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        double absoluteAngle = getAbsoluteAngle();
        steerBuiltinEncoder.setPosition(absoluteAngle);
        currentAngleRadians = absoluteAngle;
      }
    } else {
      resetIteration = 0;
    }

    double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
    if (currentAngleRadiansMod < 0.0) {
      currentAngleRadiansMod += 2.0 * Math.PI;
    }

    // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
    double adjustedReferenceAngleRadians =
        referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
    if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
      adjustedReferenceAngleRadians -= 2.0 * Math.PI;
    } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
      adjustedReferenceAngleRadians += 2.0 * Math.PI;
    }

    this.referenceAngleRadians = referenceAngleRadians;

    steerMotorController.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
  }

  /** the current steer angle in radians */
  public double getSteerAngle() {
    double motorAngleRadians = steerBuiltinEncoder.getPosition();
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < 0.0) {
      motorAngleRadians += 2.0 * Math.PI;
    }

    return motorAngleRadians;
  }


  /** The current velocity in meters per second */
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getDistance() {
    return driveEncoder.getPosition();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistance(), new Rotation2d(getSteerAngle()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
  }

  public void set(double driveVoltage, double steerAngle) {
    if (driveVoltage == 0.0 && steerAngle != Math.toRadians(DriveConstants.kXModeAngle)) {
      driveMotor.setVoltage(driveVoltage);
      return;
    }
    steerAngle %= (2.0 * Math.PI);
    if (steerAngle < 0.0) {
      steerAngle += 2.0 * Math.PI;
    }

    double difference = steerAngle - getSteerAngle();
    // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
    if (difference >= Math.PI) {
      steerAngle -= 2.0 * Math.PI;
    } else if (difference < -Math.PI) {
      steerAngle += 2.0 * Math.PI;
    }
    difference = steerAngle - getSteerAngle(); // Recalculate difference

    // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so
    // the total
    // movement of the module is less than 90 deg
    if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
      // Only need to add 180 deg here because the target angle will be put back into the range [0,
      // 2pi)
      steerAngle += Math.PI;
      driveVoltage *= -1.0;
    }

    // Put the target angle back into the range [0, 2pi)
    steerAngle %= (2.0 * Math.PI);
    if (steerAngle < 0.0) {
      steerAngle += 2.0 * Math.PI;
    }

    driveMotor.setVoltage(driveVoltage);
    setReferenceAngle(steerAngle);
  }
}
