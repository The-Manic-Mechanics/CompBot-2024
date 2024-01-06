// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// FIXME: This entire file needs to be reworked.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
* Holds all things pertaining to the LimeLight
*/
public final class LimeLight extends SubsystemBase {
	/**
	* The limelight data from the Network Table
	* See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
	*/
	private static final NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

	/**
	* The horizontal offset from the limelight crosshair to the target
	* See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
	*/
	private final static NetworkTableEntry
			tX = limeLightTable.getEntry("tx"),

            /**
             * The limelight distance from target
             * (Read as area that the target takes up
             * (If you still don't know remember that objects diminish with distance))
             * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
             */
	        tA = limeLightTable.getEntry("ta"),

	        /**
	        * The april tag id identified by the limelight
	        * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
	        */
	        tID = limeLightTable.getEntry("tid"),

			/**
			 * Current position of the robot on the field in (X, Y, Z)
			 * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
			 */
			botPose = limeLightTable.getEntry("botpose");
	/**
    * The vertical offset from the limelight crosshair to the target
    * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
    */
	public final static NetworkTableEntry tY = limeLightTable.getEntry("ty");

    /**
    * Holds the bot pose calculated based off of the april tag (X, Y, Z, Roll, Pitch, and Yaw respectivly)
    * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
    */
	public static double[] botPoseArray;

	boolean tagDetected;
	// TODO: Make sure this is updating properly.
	public static double id;

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// Getting the Limelight values from the Network tables periodically
		id = tID.getDouble(9.0);

		botPoseArray = botPose.getDoubleArray(new double[6]);

		// Putting LimeLight values onto SmartDashboard
		SmartDashboard.putNumber("LimeLight X", tX.getDouble(0.0));
		SmartDashboard.putNumber("LimeLight Y", tY.getDouble(0.0));
		SmartDashboard.putNumber("LimeLight Area", tA.getDouble(0.0));
		// SmartDashboard.putString("Currently Viewed AprilTag", currentlyViewedAprilTag);
		SmartDashboard.putNumber("BotPose Z", botPoseArray[2]);

		SmartDashboard.putBoolean("AprilTag Detected", tagDetected);
		//SmartDashboard.putString("Currently Viewed AprilTag", currentlyViewedAprilTag);

	}

    /**
     * @return The bot pose as a <b>Translation2d</b>
    */
	public static Translation2d GetBotPose2d() {
		return new Translation2d(botPoseArray[0], botPoseArray[1]);
	}
//    /**
//     * @return The distance from the bot to the april tag
//    */
//  public static double GetPOIDistance() {
// 		return (LimeLightOffsets.aprilTagToFloorInches - LimeLightOffsets.lensHeightInches) / Math.tan(LimeLightOffsets.angleToAprilTagRadians);
// 	}
//
// 	public static double GetCurrentAprilTag() {
// 		return tID.getDouble(9.0);
// 	}
//
// 	public static double GetTX() {
// 		return tX.getDouble(42);
// 	}

}
