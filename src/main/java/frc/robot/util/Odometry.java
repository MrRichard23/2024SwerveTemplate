package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import static frc.robot.util.Constants.DrivetrainSpecs.*;
import static frc.robot.util.Constants.LimelightPipeline.*;
import static frc.robot.util.Constants.VisionConstants.*;

public class Odometry {

    private Drivetrain drivetrain;
    private Limelight limelight;

    public final SwerveDriveKinematics driveKinematics;
	  private SwerveDriveOdometry odometry;

    public Odometry(){
        drivetrain = Drivetrain.getInstance();
        limelight = Limelight.getInstance();

        driveKinematics = drivetrain.getDriveKinematics();

        odometry = new SwerveDriveOdometry(driveKinematics, drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions());

    }

    public SwerveDriveOdometry getOdometry () {
        return odometry;
    }

    public Pose2d getDriveKinematics () {
        return odometry.getPoseMeters();
    }

    public void resetLimelightOdometry() {
      odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), robotToField(limelight.getLimelightPose()));
    }

    public void resetOdometry(Pose2d pose) {
      odometry.resetPosition(drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions(), robotToField(pose));
  }

  public void resetOdometry(double x, double y, double theta) {
    resetOdometry(new Pose2d(x, y, new Rotation2d(Math.toRadians(theta))));
  }

    public void updateOdometry () {
      odometry.update(Rotation2d.fromDegrees(drivetrain.getNavX().getFusedHeading()), drivetrain.getModulePositions());
    }

  // TODO

  public Pose2d robotToField (Pose2d pose) {
    return pose.transformBy(new Transform2d(new Translation2d(ROBOT_TO_FIELDX, ROBOT_TO_FIELDY), new Rotation2d(0)));
  }
}
