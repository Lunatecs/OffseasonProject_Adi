/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Map;

public class TrackToTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final Map<Integer, Pose2d> tagMap;

    public TrackToTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, Map<Integer, Pose2d> blueMap, Map<Integer, Pose2d> redMap) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        this.tagMap = (alliance == Alliance.Red) ? redMap : blueMap;

        addRequirements(drivetrain, limelight);
    }

    private Pose2d getClosestTagPose(Pose2d currentPose) {
        return tagMap.values().stream()
            .min((a, b) -> Double.compare(
                a.getTranslation().getDistance(currentPose.getTranslation()),
                b.getTranslation().getDistance(currentPose.getTranslation())
            ))
            .orElse(null);
    }

    @Override
    public void execute() {
        LimelightSubsystem.LimelightPose vision = limelight.getTargetPose();
        if (!vision.valid) {
            drivetrain.stop();
            return;
        }

        Pose2d currentPose = vision.pose;
        Pose2d targetPose = getClosestTagPose(currentPose);

        if (targetPose != null) {
            Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());
            Rotation2d headingError = targetPose.getRotation().minus(currentPose.getRotation());

            double kP = 1.2;
            double kRot = 3.0;
            double vx = kP * delta.getX();
            double vy = kP * delta.getY();
            double omega = kRot * headingError.getRadians();

            vx = Math.max(-2.0, Math.min(2.0, vx));
            vy = Math.max(-2.0, Math.min(2.0, vy));
            omega = Math.max(-3.0, Math.min(3.0, omega));

            drivetrain.driveFieldRelative(vx, vy, omega);
        } else {
            drivetrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        LimelightSubsystem.LimelightPose vision = limelight.getTargetPose();
        if (!vision.valid) return true;

        Pose2d currentPose = vision.pose;
        Pose2d targetPose = getClosestTagPose(currentPose);
        if (targetPose == null) return true;

        double positionError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        return positionError < 0.2 && angleError < Math.toRadians(5);
    }
}
 */
