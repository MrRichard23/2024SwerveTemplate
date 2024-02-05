package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import static frc.robot.util.Constants.DrivetrainSpecs.*;

public class Odometry {

    private Drivetrain drivetrain;
    private Limelight limelight;

    public final SwerveDriveKinematics driveKinematics;
	private SwerveDriveOdometry odometry;

    public Odometry(){
        drivetrain = Drivetrain.getInstance();
        limelight = Limelight.getInstance();

        driveKinematics = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

        odometry = new SwerveDriveOdometry(driveKinematics, drivetrain.getGyroscopeRotation(), drivetrain.getModulePositions());

    }

    public SwerveDriveOdometry getDriveOdometry () {
        return odometry;
    }

    public Pose2d getDriveKinematics () {
        return odometry.getPoseMeters();
    }

    public void updateOdometry () {
        odometry.update(Rotation2d.fromDegrees(drivetrain.getNavX().getFusedHeading()), drivetrain.getModulePositions());
    }
}
