// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Interface to the LimeLight camera.
 */
public final class LimeLight extends SubsystemBase {
	/**
	 * The LimeLight data gathered from the NetworkTable.
	 * See <a href=
	 * "https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">the
	 * API</a>.
	 */
	private static final NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

	/**
	 * The horizontal offset from the LimeLight crosshair to the target.
	 * See <a href=
	 * "https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">the
	 * API</a>.
	 */
	private static double offsetFromCrosshairH;
	/**
	 * The LimeLight's distance from it's current target.
	 * See <a href=
	 * "https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">the
	 * API</a>.
	 */
	private static double distanceFromTarget;
	/**
	 * The vertical offset from the LimeLight crosshair to the target.
	 * See <a href=
	 * "https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">the
	 * API</a>.
	 */
	private static double offsetFromCrosshairV;

	/**
	 * Holds the BotPose calculated based off of the AprilTag (X, Y, Z, Roll,
	 * Pitch, and Yaw respectively).
	 * See <a href=
	 * "https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">the
	 * API</a>.
	 */
	public static double[] botPoseArray;
	/**
	 * The AprilTag id identified by the LimeLight.
	 * See <a href=
	 * "https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">the
	 * API</a>.
	 */
	public static double tagID;

	@Override
	public void periodic() {
		// Fetch data from the LimeLight NetworkTable.
		offsetFromCrosshairH = limeLightTable.getEntry("tx").getDouble(0.0);
		offsetFromCrosshairV = limeLightTable.getEntry("ty").getDouble(0.0);
		distanceFromTarget = limeLightTable.getEntry("ta").getDouble(0.0);
		tagID = limeLightTable.getEntry("tid").getDouble(0.0);
		
		// Getting the BotPose from the LimeLight based off of the AprilTag's position.
		botPoseArray = limeLightTable.getEntry("botpose").getDoubleArray(new double[6]);

		// Uncomment to put debug values on the SmartDashboard.

		// SmartDashboard.putNumber("LimeLight X", offsetFromCrosshairH);
		// SmartDashboard.putNumber("LimeLight Y", offsetFromCrosshairV);
		// SmartDashboard.putNumber("LimeLight Area", distanceFromTarget);
		// SmartDashboard.putNumber("BotPose Z", botPoseArray[2]);
	}

	/**
	 * @return The BotPose as a Pose2d instance.
	 */
	public static Pose2d getBotPose2d() {
		// TODO: Figure out whether these values make sense in the context they're used.
		return new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(botPoseArray[5]));
	}
}