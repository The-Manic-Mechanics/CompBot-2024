// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain.*;

/**
* The subsystem that holds all the motors and functions pertaining to the DriveTrain
*/
public final class DriveTrain extends SubsystemBase {
	/**
	* Represents the MecanumDrive (Used for driving the robot)
	*/
	public static MecanumDrive mecanum;

	/**
	* Used for keeping track of the robot's position while it drives
	*/
	public static MecanumDriveOdometry mecanumDriveOdometry;

    /**
    * Stores the position of the wheels in robot space (Used for kinematics and odometry)
    */
	private static MecanumDriveWheelPositions wheelPositions;

	/**
	* Used for tracking how far the robot actually goes compared to what values are being inputed
	*/
	private static MecanumDriveKinematics mecanumDriveKinematics;

	public static class Motors {
		public static CANSparkMax frontLeft, frontRight, backLeft, backRight;
	}

	public static class Encoders {
		public static Encoder frontLeft, frontRight, backLeft, backRight;
	}

	public DriveTrain() {
		Motors.frontLeft = new CANSparkMax(Constants.Motors.Ports.DriveTrain.FRONT_LEFT, MotorType.kBrushless);
		Motors.frontRight = new CANSparkMax(Constants.Motors.Ports.DriveTrain.FRONT_RIGHT, MotorType.kBrushless);
		Motors.backLeft = new CANSparkMax(Constants.Motors.Ports.DriveTrain.BACK_LEFT, MotorType.kBrushless);
		Motors.backRight = new CANSparkMax(Constants.Motors.Ports.DriveTrain.BACK_RIGHT, MotorType.kBrushless);

		Motors.frontLeft.getAlternateEncoder(0);

		Motors.frontLeft.setInverted(true);
		Motors.backLeft.setInverted(true);

		mecanum = new MecanumDrive(Motors.frontLeft, Motors.backLeft, Motors.frontRight, Motors.backRight);

		mecanumDriveKinematics = new MecanumDriveKinematics
	    (
			new Translation2d(MotorLocations.FRONT_LEFT, MotorLocations.FRONT_LEFT),
			new Translation2d(MotorLocations.FRONT_RIGHT, -1 * MotorLocations.FRONT_RIGHT),
			new Translation2d(-1 * MotorLocations.BACK_LEFT, MotorLocations.BACK_LEFT),
			new Translation2d(-1 * MotorLocations.BACK_RIGHT, -1 * MotorLocations.BACK_RIGHT)
		);

		// wheelPositions = new MecanumDriveWheelPositions(
		// 		Encoders.frontLeft.getDistance(),
		// 		Encoders.frontRight.getDistance(),
		// 		Encoders.backLeft.getDistance(),
		// 		Encoders.backRight.getDistance()
		// );

		// double[] currentPose = LimeLight.botPoseArray;

		// Pose2d initPose = new Pose2d(currentPose[1], currentPose[2], Gyroscope.sensor.getRotation2d());

		// mecanumDriveOdometry = new MecanumDriveOdometry(mecanumDriveKinematics, Gyroscope.sensor.getRotation2d(), wheelPositions, initPose);
	}

    /**
     * Gets the current mecanum wheel positions.
	 *
	 * @return <i>(type MecanumDriveWheelPositions)</i> filled out
    */
	// public static MecanumDriveWheelPositions getWheelPositions() {
	// 	return new MecanumDriveWheelPositions(
	// 			Encoders.frontLeft.getDistance(), Encoders.frontRight.getDistance(),
	// 			Encoders.backLeft.getDistance(), Encoders.backRight.getDistance());
	// }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// mecanumDriveOdometry.update(Gyroscope.sensor.getRotation2d(), wheelPositions);

		// DEBUG: SmartDashboard entries
		SmartDashboard.putNumber("X Value", RobotContainer.driverOneController.getLeftX());
		SmartDashboard.putNumber("Y Value", RobotContainer.driverOneController.getLeftY());
		SmartDashboard.putNumber("Z Value", RobotContainer.driverOneController.getRightX());
	}
}
