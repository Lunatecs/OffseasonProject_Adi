package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final String cameraName;
    private Pose2d lastPose = new Pose2d();
    private boolean lastValid = false;

    public LimelightSubsystem(String cameraName) {
        this.cameraName = cameraName;
    }

    public static class LimelightPose {
        public final Pose2d pose;
        public final boolean valid;

        public LimelightPose(Pose2d pose, boolean valid) {
            this.pose = pose;
            this.valid = valid;
        }
    }

    public LimelightPose getTargetPose() {
        NetworkTable nt = NetworkTableInstance.getDefault().getTable(cameraName);
        double[] targetPose = nt.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        double tagCount = nt.getEntry("tc").getDouble(0.0);
        double ambiguity = nt.getEntry("ta3").getDouble(1.0);

        if (targetPose.length < 6 || tagCount < 1 || ambiguity > 0.3) {
            lastValid = false;
            return new LimelightPose(lastPose, false);
        }

        Translation2d targetTranslation = new Translation2d(targetPose[0], targetPose[1]);
        Rotation2d targetRotation = Rotation2d.fromDegrees(targetPose[5]);
        lastPose = new Pose2d(targetTranslation, targetRotation);
        lastValid = true;
        return new LimelightPose(lastPose, true);
    }

    public boolean hasValidPose() {
        return lastValid;
    }

    public Pose2d getLastPose() {
        return lastPose;
    }
}
