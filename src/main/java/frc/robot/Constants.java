package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double DT = 0.02; // 20ms framerate 50Hz
  public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz

  public static final class SelectorConstants {
    public static final int kSelectorChannel = 3;
  }

  public static final class LEDconstants {
    public static final int port_number = 0;
  }

  public static final class PneumaticConstants {
    public static final int kSolenoidChannel = 3;
  }

  public static final class HopperConstants {
    // motor ports
    public static final int kIntakeMotorPort = 7;
    public static final int kHopperMotorPort = 12;
    public static final int kVerticalMotorPort = 18;
    public static final int kGateMotorPort = 2;
    // sensor ports
    public static final int kIntakeLineBreakSensor = 2;
    public static final int kHopperLineBreakSensor = 1;
    public static final int kVerticalLineBreakSensor = 0;
    // motor speeds
    public static final double kHopperSpeed = -0.6;
    public static final double kIntakeSpeed = 0.9;
    public static final double kVerticalSpeed = 0.25;
    public static final double kAutoVert = 0.10;
    public static final double kFeedSpeed = 0.9;
    public static final double kLowFeed = 0.5;
    public static final double kGateSpeed = 0.25;

    // pnematic config
    public static final int kIntakeChannel = 0;
    public static final int kPneumaticHubPort = 20;
  }

  public static final class ShooterConstants {
    // motor ports
    public static final int kShooterMotorOnePort = 16;
    public static final int kShooterMotorTwoPort = 17;
    // motor speeds
    public static final double kShooterMotorOneSpeed = 0.1;

    // other values
    public static final double kShooterTolerance = 0;
    public static final double kSHooterTarget = 0;

    public static final double kLowPortRPM = 800;
  }

  public static final class DriveConstants {

    // Track Width Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.4445;
    // Wheel Base Distance between front and back wheels on robot
    public static final double kWheelBase = 0.7334;

    // NEO motor free spin max from documentation
    public static final double MAX_MOTOR_FREE_SPEED_RPM = 5676.0;

    // Steer drive gear reduction for SDS MK3 module
    public static final double kSteerDriveReduction = (15.0 / 32.0) * (10.0 / 60.0);

    // SwerveDriveSpecialties standard gear reduction (was
    // SdsModuleConfigurations.MK3_STANDARD.getDriveReduction())
    public static final double kDriveGearReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);
    // diameter of the drive wheels in meters (was
    // SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter())
    public static final double kWheelDiameterMeters = 0.1016;

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        MAX_MOTOR_FREE_SPEED_RPM / 60.0 * kDriveGearReduction * kWheelDiameterMeters * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

    public static final double MAX_VOLTAGE = 12.0;

    // value in amps to limit the neo motor with setSmartCurrentLimit
    public static final int kDriveCurrentLimit = 40;

    // value in volts to set the voltage compensation on the SPARK MAX via
    // enableVoltageCompensation
    public static final double kNominalVoltage = 12.0;

    // value in amps to limit the neo motor with setSmartCurrentLimit
    public static final int kSteerCurrentLimit = 20;

    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kBackLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kBackRightDriveMotorPort = 14;

    public static final int kFrontLeftTurningMotorPort = 9;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 11;
    public static final int kBackRightTurningMotorPort = 15;

    public static final int kFrontLeftTurningInputPort = 28;
    public static final int kBackLeftTurningInputPort = 25;
    public static final int kFrontRightTurningInputPort = 26;
    public static final int kBackRightTurningInputPort = 27;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final double kFrontLeftZeroAngle = 104.89;
    public static final double kBackLeftZeroAngle = 272.0;
    public static final double kFrontRightZeroAngle = 303.86;
    public static final double kBackRightZeroAngle = 199.92;

    public static final double kFrontLeftEncoderAngle = -169.722; // -165.322 ;
    public static final double kBackLeftEncoderAngle = -24.53; // -26.63;
    public static final double kFrontRightEncoderAngle = -12.2; // -12.47 //-11.688;
    public static final double kBackRightEncoderAngle = -65.1; // -65.37

    public static final double kXModeAngle = 135.0;

    public enum PivotPoint {
      CENTER(new Translation2d(0.0, 0.0)),
      FRONT_LEFT(new Translation2d(kTrackWidth / 2, kWheelBase / 2)),
      FRONT_RIGHT(new Translation2d(-kTrackWidth / 2, kWheelBase / 2)),
      BACK_RIGHT(new Translation2d(kTrackWidth / 2, -kWheelBase / 2)),
      BACK_LEFT(new Translation2d(-kTrackWidth / 2, -kWheelBase / 2));

      private final Translation2d m_pivotPoint;

      PivotPoint(Translation2d pivotPoint) {
        this.m_pivotPoint = pivotPoint;
      }

      public Translation2d get() {
        return m_pivotPoint;
      }

      public static PivotPoint getByPOV(int pov) {
        switch (pov) {
          case 0:
            return PivotPoint.FRONT_LEFT;
          case 90:
            return PivotPoint.FRONT_RIGHT;
          case 180:
            return PivotPoint.BACK_RIGHT;
          case 270:
            return PivotPoint.BACK_LEFT;
          default:
            return PivotPoint.CENTER;
        }
      }
    }

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // back left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right

    public static int kPigeon2_CANid = 24;

    public static final double kMaxSpeedMetersPerSecond = MAX_VELOCITY_METERS_PER_SECOND;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond =
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxDecelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 10;
    public static final double kTeleDriveMaxAngularDecelerationUnitsPerSecond = 20;
  }

  public static final class ClimberConstants {
    public static final int kLeftClimbMotorPort = 13;
    public static final int kRightClimbMotorPort = 6;

    // climb solenoid
    public static final int kCLimbSolenoidPort = 1;

    public static final double kMaxHeight = -41.88;
    public static final double kMinHeight = 0.0;
    public static final double kMidClimb = -32.0;
    public static final double kLowBar = -13.0;

    public static final double kBeforeExtendHeight = -10;
    public static final double kToNextBar = -43;
    public static final double kOnNextBar = 0;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI * 4;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared =
        2 * Math.PI * 16;

    // public static final int kEncoderCPR = 1024;
    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = Units.feetToMeters(4); // 0.1016;
    public static final double kDriveGearReduction = 6.67 / 1.0;
    public static final double kDriveEncoderRotationsPerMeter =
        kWheelDiameterMeters * Math.PI / kDriveGearReduction;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * kDriveGearReduction / (double) kEncoderCPR;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRotationsPerMeter / 60;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = .1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2; // 4 kinda worked// .4;
    public static final double kPYController = 2; // .4;
    public static final double kPThetaController = 8; // 4;

    // Constraint for the motion profilied robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new
    // TrapezoidProfile.Constraints(
    //    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    // PID values for the rotation in the TargetDrive command
    public static final double kPTargetTurn = .01;
    public static final double kITargetTurn = 0.001;
    public static final double kDTargetTurn = 0.0;

    // P value used in the AutoRotate command
    public static final double kPAutoTurn = .01;
  }
}
