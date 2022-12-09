package frc.robot;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;


public class PoseEstimatorSubsystem extends SubsystemBase {
    
    private final PhotonCamera photonCamera;
    private final SwerveDrivePoseEstimator<N7, N7, N5> poseEstimator;
    private final DriveSubsystem driveSubsystem;

    private final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Pose3d( 0, 0, 0, new Rotation3d( 0, 0, 0)),
        new Pose3d( 0, .02, 1.0, new Rotation3d( 0, 0, 0)));

    private final List<Pose3d> tagPoseList = Collections.unmodifiableList(List.of(
        new Pose3d(2.0, 2.0, 1.0, new Rotation3d(0, 0, Units.degreesToRadians(180.0))),
        new Pose3d(1.0, 3.0, 1.0, new Rotation3d(0, 0, Units.degreesToRadians(180.0)))
    ));

    private static final Vector<N7> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);
    private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public PoseEstimatorSubsystem(PhotonCamera photonCamera, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.photonCamera = photonCamera;

        poseEstimator = new SwerveDrivePoseEstimator<N7, N7, N5>(Nat.N7(), Nat.N7(), Nat.N5(),
                                Rotation2d.fromDegrees(driveSubsystem.getHeading()),
                                driveSubsystem.getModulePositions(),
                                new Pose2d(),
                                Constants.DriveConstants.kDriveKinematics,
                                stateStdDevs,
                                localMeasurementStdDevs,
                                visionMeasurementStdDevs);

    }

    @Override
    public void periodic() {
        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();

        if (pipelineResult.hasTargets()) {
            List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
            for (PhotonTrackedTarget target: targets) {
                if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() < 0.2) {
                    int fiducialId = target.getFiducialId();
                    Pose3d targetPose = tagPoseList.get(fiducialId);
                    Transform3d camToTarget = target.getBestCameraToTarget();
                    Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
            
                    var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
                    poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);


                }
            }
        }

    }


}
