package frc.robot.subsystems;

import java.util.AbstractMap;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class PoseEstimatorSubsystem extends SubsystemBase {
    
    private final PhotonCamera photonCamera;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final DriveSubsystem driveSubsystem;

    private final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Pose3d( 0, 0, 0, new Rotation3d( 0, 0, 0)),
        new Pose3d( 0, .02, 1.0, new Rotation3d( 0, 0, 0)));

    private final List<Pose3d> tagPoseList = Collections.unmodifiableList(List.of(
        new Pose3d(2.0, 2.0, 1.0, new Rotation3d(0, 0, Units.degreesToRadians(180.0))),
        new Pose3d(1.0, 3.0, 1.0, new Rotation3d(0, 0, Units.degreesToRadians(180.0)))
    ));

    private final Map<Integer, Pose3d> tagMap = Map.ofEntries(
        new AbstractMap.SimpleEntry<Integer, Pose3d>(0, new Pose3d(0, 4.11, 0.768, new Rotation3d(0, 0, Units.degreesToRadians(0.0)))),
        new AbstractMap.SimpleEntry<Integer, Pose3d>(2, new Pose3d(4.11, 0.0, 0.768, new Rotation3d(0, 0, Units.degreesToRadians(90.0)))),
        new AbstractMap.SimpleEntry<Integer, Pose3d>(3, new Pose3d(4.11, 8.22, 0.768, new Rotation3d(0, 0, Units.degreesToRadians(270.0)))),
        new AbstractMap.SimpleEntry<Integer, Pose3d>(10, new Pose3d(4.11, 4.11, 1.5, new Rotation3d(0, 0, Units.degreesToRadians(90.0)))),
        new AbstractMap.SimpleEntry<Integer, Pose3d>(11, new Pose3d(4.11, 4.11, 1.5, new Rotation3d(0, 0, Units.degreesToRadians(0.0)))),
        new AbstractMap.SimpleEntry<Integer, Pose3d>(12, new Pose3d(4.11, 4.11, 1.5, new Rotation3d(0, 0, Units.degreesToRadians(270.0)))),
        new AbstractMap.SimpleEntry<Integer, Pose3d>(13, new Pose3d(4.11, 4.11, 1.5, new Rotation3d(0, 0, Units.degreesToRadians(180.0))))
    );
    private static final Matrix<N3,N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Matrix<N3,N1> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private final Field2d field2d = new Field2d();
    
    private final DataLog dataLog;
    private final BooleanLogEntry hasTargetsLog;
    private final IntegerLogEntry fiducialIdLog;
    private final StringLogEntry tagLog; 



    public PoseEstimatorSubsystem(PhotonCamera photonCamera, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.photonCamera = photonCamera;

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        
        poseEstimator = new SwerveDrivePoseEstimator(
                                Constants.DriveConstants.kDriveKinematics,
                                driveSubsystem.getRotation(),
                               // Rotation2d.fromDegrees(driveSubsystem.getHeading()),
                                driveSubsystem.getModulePositions(),
                                new Pose2d(),
                                stateStdDevs,
                                visionMeasurementStdDevs);

        tab.addString("Pose", this::getPoseString).withPosition(0, 0).withSize(2, 0);
 //       tab.addDouble("AngleTo0", this::getAngleToTarget0).withPosition(0, 2).withSize(2, 0);
        tab.addBoolean("has targets", this::hasTargets).withPosition(0,2).withSize(2,1);
        tab.addInteger("best target Id", this::bestFiducialId).withPosition(0,3).withSize(2,1);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4)
                  .withWidget(BuiltInWidgets.kField);
                            
        dataLog = DataLogManager.getLog();
        hasTargetsLog = new BooleanLogEntry(dataLog,"/pose/hastargets");
        fiducialIdLog = new IntegerLogEntry(dataLog, "/pose/fiducialId");
        tagLog = new StringLogEntry(dataLog, "/pose/visionmeasurement");
    }

    private void logTarget(PhotonTrackedTarget target) {
        fiducialIdLog.append(target.getFiducialId());
        DoubleLogEntry ambiguityLogEntry = new DoubleLogEntry(dataLog, "/pose/ambiguity"+target.getFiducialId());
        ambiguityLogEntry.append(target.getPoseAmbiguity());
        DoubleLogEntry transformXLogEntry = new DoubleLogEntry(dataLog, "/pose/X"+target.getFiducialId());
        transformXLogEntry.append(target.getBestCameraToTarget().getX());
        DoubleLogEntry transformYLogEntry = new DoubleLogEntry(dataLog, "/pose/Y"+target.getFiducialId());
        transformYLogEntry.append(target.getBestCameraToTarget().getY());
    }

    private boolean hasTargets;
    public boolean hasTargets() {
        return hasTargets;
    }

    private int bestFiducialId;
    public int bestFiducialId() {
        return bestFiducialId;
    }
    @Override
    public void periodic() {
        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();
        bestFiducialId = -1;

        hasTargets = pipelineResult.hasTargets();
        hasTargetsLog.append(pipelineResult.hasTargets());
        if (pipelineResult.hasTargets()) {
            PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
            bestFiducialId = bestTarget.getFiducialId();
            List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
            for (PhotonTrackedTarget target: targets) {
                logTarget(target);
                if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() < 0.2) {
                    int fiducialId = target.getFiducialId();
                    Pose3d targetPose = tagMap.get(fiducialId);
                    if ( targetPose != null ) {
                        Transform3d camToTarget = target.getBestCameraToTarget();
                        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
                        //Pose3d visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
                        Pose3d visionMeasurement = camPose;
                        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
                        tagLog.append("Added vision measurement ["+visionMeasurement.getX() + ","+visionMeasurement.getY()+","+visionMeasurement.getRotation().toRotation2d().getDegrees()+"]");
                    }
                }
            }
        }
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
            driveSubsystem.getRotation(),
            driveSubsystem.getModulePositions());
  
        field2d.setRobotPose(getPose());

        Pose3d target0 = tagMap.get(0);
       // Rotation2d targetAngle = getAngleToPose(target0);
       

        // get all the angles ....
        //for (Pose3d target : tagMap.values()) {
        //    Rotation2d angleToTarget = getAngleToPose(target);
        //}
    }

 //   public double getAngleToTarget0() {
 //       Pose3d target0 = tagMap.get(0);
 //       Rotation2d targetAngle = getAngleToPose(target0);
 //       return targetAngle.getDegrees();
 //   }

//    public Rotation2d getAngleToPose(Pose3d targetPose) {
//        Pose3d myPose = new Pose3d(getPose());
//        Transform3d transform = new Transform3d(myPose, targetPose);
//        return transform.getRotation().toRotation2d();
//    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
          driveSubsystem.getRotation(),
          driveSubsystem.getModulePositions(),
          newPose);
    }

    public String getPoseString() {
        Pose2d pose = getPose();
        return String.format("(%.2f, %.2f) %.2f degrees", 
            pose.getX(), 
            pose.getY(),
            pose.getRotation().getDegrees());    
    }
}
