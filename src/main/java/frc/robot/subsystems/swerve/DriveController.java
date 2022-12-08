package frc.robot.subsystems.swerve;

import static frc.robot.util.RevUtil.checkRevError;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.DriveConstants;

public class DriveController {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public DriveController(int id) {
    motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor.setInverted(true);

    // Setup voltage compensation
    checkRevError(
        motor.enableVoltageCompensation(DriveConstants.kNominalVoltage),
        "Failed to enable voltage compensation");

    checkRevError(
        motor.setSmartCurrentLimit(DriveConstants.kDriveCurrentLimit),
        "Failed to set current limit for NEO");

    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
        "Failed to set periodic status frame 0 rate");
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
        "Failed to set periodic status frame 1 rate");
    checkRevError(
        motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
        "Failed to set periodic status frame 2 rate");
    // Set neutral mode to brake
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Setup encoder
    encoder = motor.getEncoder();
    double positionConversionFactor =
        Math.PI * DriveConstants.kWheelDiameterMeters * DriveConstants.kDriveGearReduction;
    encoder.setPositionConversionFactor(positionConversionFactor);
    encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
  }

  public void setReferenceVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public double getStateVelocity() {
    return encoder.getVelocity();
  }

  public double getDistance() {
    return encoder.getPosition();
  }
}
