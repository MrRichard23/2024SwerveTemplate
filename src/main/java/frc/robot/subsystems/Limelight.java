package frc.robot.subsystems;

import frc.robot.util.Constants.LimelightPipeline;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.estimator.PoseEstimator;

public class Limelight extends SubsystemBase {
	private static Limelight limelight;
	private static NetworkTable llTable;

	private Limelight() {
		llTable = NetworkTableInstance.getDefault().getTable("limelight");
		
	}


	public void setPipeline(LimelightPipeline pipe) {
		llTable.getEntry("pipeline").setNumber(pipe.id());
	}

	public LimelightPipeline getPipeline() {
		int id = (int) llTable.getEntry("pipeline").getInteger(0);

		if (id == 0) {
			return LimelightPipeline.APRILTAG;
		} else if (id == 1) {
			return LimelightPipeline.REFLECTIVE;
		} else if (id == 2) {
			return LimelightPipeline.DRIVER;
		}

		return LimelightPipeline.DRIVER;
	}

	public boolean hasTarget() {
		return llTable.getEntry("tv").getInteger(0) == 1;
	}

	public double getTargetArea() {
		return llTable.getEntry("ta").getDouble(0);
	}

	// Translation (x,y,z) Rotation(pitch,yaw,roll)
	public Number[] getCamTran() {

		return llTable.getEntry("targetpose_robotspace").getNumberArray(new Number[0]);
	}

	// public double getX() {
	// 	return llTable.getEntry("x distance").getDouble(0);
	// }
	public double getX() {
		if(getCamTran().length < 1){
			return 0;
		}
		return (double) getCamTran()[0];
	}

	public double getY() {
		if(getCamTran().length < 1){
			return 0;
		}
		return (double) getCamTran()[1];
	}

	public double getZ() {
		if(getCamTran().length < 1){
			return 0;
		}
		return (double) getCamTran()[2];
	}

	public double getPitch() {
		if(getCamTran().length < 1){
			return 0;
		}
		return (double) getCamTran()[3];
	}

	public double getYaw() {
		if(getCamTran().length < 1){
			return 0;
		}
		return (double) getCamTran()[4];
	}

	public double getRoll() {
		if(getCamTran().length < 1){
			return 0;
		}
		return (double) getCamTran()[5];
	}

	public Pose2d getFieldPose() {
		double[] poseNum = new double[6];
		
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			poseNum = llTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		} else {
			poseNum = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		}

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));
	}

	public Pose2d getLimelightPose() {
		return new Pose2d(getX(), getY(), new Rotation2d(getYaw()));
	  }
	  

	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("limelightOdometry", (Double) getX());
	}

	public static Limelight getInstance() {
		if (limelight == null) {
			limelight = new Limelight();
		}
		return limelight;
	}
}
