// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
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
	public static double
			/**
			 * The offset of the apriltag from the Limelight crosshair (Horizontal)
			 */
			offsetFromCrosshairH,

            /**
             * The limelight distance from target
             * (Read as area that the target takes up
             * (If you still don't know remember that objects diminish with distance))
             * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
             */
	        distanceFromTarget,

	        /**
	        * The april tag id identified by the limelight
	        * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
	        */
	        tagID,
			/**
			 * Current position of the robot on the field in (X, Y, Z)
			 * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
			 */
			botPose,
			/**
    		* The vertical offset from the limelight crosshair to the target
    		* See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
    		*/
			offsetFromCrosshairV;

    /**
    * Holds the bot pose calculated based off of the april tag (X, Y, Z, Roll, Pitch, and Yaw respectivly)
    * See <a href="https://docs.limelightvision.io/en/latest/networktables_api.html?highlight=api">...</a>
    */
	public static double[] botPoseArray;

	boolean tagDetected;
	public static double id;

	@Override
	public void periodic() {
		// Fetch data from LimeLight
		offsetFromCrosshairH = limeLightTable.getEntry("tx").getDouble(0.0);
		offsetFromCrosshairV = limeLightTable.getEntry("ty").getDouble(0.0);
		distanceFromTarget = limeLightTable.getEntry("ta").getDouble(0.0);
		tagID = limeLightTable.getEntry("tid").getDouble(0.0);
		botPose = limeLightTable.getEntry("botpose").getDouble(0.0);


		// This method will be called once per scheduler run
		// Getting the Limelight values from the Network tables periodically (Default 0)
		id = tagID;

		// Getting the botpose from the limelight based off of the apriltag 
		botPoseArray = limeLightTable.getEntry("botpose").getDoubleArray(new double[6]);

		// Putting LimeLight values onto SmartDashboard
		SmartDashboard.putNumber("LimeLight X", offsetFromCrosshairH);
		SmartDashboard.putNumber("LimeLight Y", offsetFromCrosshairV);
		SmartDashboard.putNumber("LimeLight Area", distanceFromTarget);
		SmartDashboard.putNumber("BotPose Z", botPoseArray[2]);

		SmartDashboard.putBoolean("AprilTag Detected", tagDetected);

	}

     /**
     * @return The bot pose as a <b>Pose2d</b>
    */
	public static Pose2d getBotPose2d() {
		// TODO Figure out whether these values make sense in the context they're used
		return new Pose2d(botPoseArray[0], botPoseArray[1], new Rotation2d(botPoseArray[5]));
	}
}