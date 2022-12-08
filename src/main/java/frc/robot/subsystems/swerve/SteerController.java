package frc.robot.subsystems.swerve;

import static frc.robot.util.RevUtil.checkRevError;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;

public class SteerController {

  private static final int ENCODER_RESET_ITERATIONS = 500;
  private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

  private final CANSparkMax motor;
  private final SparkMaxPIDController controller;
  private final RelativeEncoder motorEncoder;
  private final CANCoder absoluteEncoder;

  private double referenceAngleRadians = 0;

  private double resetIteration = 0;

  public SteerController(int steerMotorId, int canEncoderId, double offsetAngleDegrees) {

    this.absoluteEncoder = createCANCoder(canEncoderId, offsetAngleDegrees);
    this.motor = createMotor(steerMotorId);
    this.controller = motor.getPIDController();
    this.motorEncoder = motor.getEncoder();
  }

  public CANSparkMax createMotor(int id) {

    double pidProportional = 1.0; // originally 1.0;
    double pidIntegral = 0.0;
    double pidDerivative = 0.1;

    CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
        "Failed to set periodic status frame 0 rate");
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate");
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
        "Failed to set periodic status frame 2 rate");
    checkRevError(motor.setIdleMode(CANSparkMax.IdleMode.kBrake), "Failed to set NEO idle mode");
    motor.setInverted(false);
    checkRevError(
        motor.enableVoltageCompensation(DriveConstants.kNominalVoltage),
        "Failed to enable voltage compensation");
    checkRevError(
        motor.setSmartCurrentLimit(DriveConstants.kSteerCurrentLimit),
        "Failed to set NEO current limits");

    RelativeEncoder integratedEncoder = motor.getEncoder();
    checkRevError(
        integratedEncoder.setPositionConversionFactor(
            2.0 * Math.PI * DriveConstants.kSteerDriveReduction),
        "Failed to set NEO encoder conversion factor");
    checkRevError(
        integratedEncoder.setVelocityConversionFactor(
            2.0 * Math.PI * DriveConstants.kSteerDriveReduction / 60.0),
        "Failed to set NEO encoder conversion factor");
    checkRevError(
        integratedEncoder.setPosition(getAbsoluteAngle()), "Failed to set NEO encoder position");

    SparkMaxPIDController controller = motor.getPIDController();
    checkRevError(controller.setP(pidProportional), "Failed to set NEO PID proportional constant");
    checkRevError(controller.setI(pidIntegral), "Failed to set NEO PID integral constant");
    checkRevError(controller.setD(pidDerivative), "Failed to set NEO PID derivative constant");

    checkRevError(
        controller.setFeedbackDevice(integratedEncoder), "Failed to set NEO PID feedback device");

    return motor;
  }

  public CANCoder createCANCoder(int id, double offsetDegrees) {
    Direction direction = Direction.COUNTER_CLOCKWISE;
    int periodMilliseconds = 10;

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = offsetDegrees;
    config.sensorDirection = direction == Direction.CLOCKWISE;

    CANCoder encoder = new CANCoder(id);
    checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

    checkCtreError(
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250),
        "Failed to configure CANCoder update rate");

    return encoder;
  }

  public double getAbsoluteAngle() {
    double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition());
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    ErrorCode errorCode = absoluteEncoder.getLastError();
    if (errorCode != ErrorCode.OK) {
      checkCtreError(errorCode, "Last CANCoder Error");
    }

    CANCoderStickyFaults faults = new CANCoderStickyFaults();
    checkCtreError(absoluteEncoder.getStickyFaults(faults), " Error getting sticky faults");
    if (faults.hasAnyFault()) {
      DriverStation.reportError(String.format("CANCoder fault: %s", faults.toString()), false);
      absoluteEncoder.clearStickyFaults();
    }

    return angle;
  }

  public double getReferenceAngle() {
    return referenceAngleRadians;
  }

  public void setReferenceAngle(double referenceAngleRadians) {
    double currentAngleRadians = motorEncoder.getPosition();

    // Reset the NEO's encoder periodically when the module is not rotating.
    // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and
    // we don't
    // end up getting a good reading. If we reset periodically this won't matter anymore.
    if (motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
      if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
        resetIteration = 0;
        double absoluteAngle = getAbsoluteAngle();
        motorEncoder.setPosition(absoluteAngle);
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

    controller.setReference(adjustedReferenceAngleRadians, CANSparkMax.ControlType.kPosition);
  }

  public double getStateAngle() {
    double motorAngleRadians = motorEncoder.getPosition();
    motorAngleRadians %= 2.0 * Math.PI;
    if (motorAngleRadians < 0.0) {
      motorAngleRadians += 2.0 * Math.PI;
    }

    return motorAngleRadians;
  }

  public static void checkCtreError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
    }
  }

  public enum Direction {
    CLOCKWISE,
    COUNTER_CLOCKWISE
  }
}
