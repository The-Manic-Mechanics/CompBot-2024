// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain.*;
import frc.robot.Constants.Auton;

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
		public static /* WPI_VictorSPX */ Spark frontLeft, frontRight, backLeft, backRight;
	}

	public static class Encoders {
		public static Encoder frontLeft, frontRight, backLeft, backRight;
	}

	public DriveTrain() {
		Motors.frontLeft = new Spark(MotorPorts.FRONT_LEFT);
		Motors.frontRight = new Spark(MotorPorts.FRONT_RIGHT);
		Motors.backLeft = new Spark(MotorPorts.BACK_LEFT);
		Motors.backRight = new Spark(MotorPorts.BACK_RIGHT);

		Encoders.frontLeft = new Encoder(
				EncoderPorts.FRONT_LEFT_A,
				EncoderPorts.FRONT_LEFT_B
		);

		Encoders.frontRight = new Encoder(
				EncoderPorts.FRONT_RIGHT_A,
				EncoderPorts.FRONT_RIGHT_B
		);

		Encoders.backLeft = new Encoder(
				EncoderPorts.BACK_LEFT_A,
				EncoderPorts.BACK_LEFT_A
		);

		Encoders.backRight = new Encoder(
				EncoderPorts.BACK_RIGHT_A,
				EncoderPorts.BACK_RIGHT_B
		);
		Encoders.frontLeft.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.frontRight.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.backLeft.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.backRight.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);

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

		wheelPositions = new MecanumDriveWheelPositions(
				Encoders.frontLeft.getDistance(),
				Encoders.frontRight.getDistance(),
				Encoders.backLeft.getDistance(),
				Encoders.backRight.getDistance()
		);

		double[] currentPose = LimeLight.botPoseArray;

		Pose2d initPose = new Pose2d(currentPose[1], currentPose[2], Gyroscope.sensor.getRotation2d());

		mecanumDriveOdometry = new MecanumDriveOdometry(mecanumDriveKinematics, Gyroscope.sensor.getRotation2d(), wheelPositions, initPose);
	}

    /**
     * Gets the current mecanum wheel positions.
	 *
	 * @return <i>(type MecanumDriveWheelPositions)</i> filled out
    */
	public static MecanumDriveWheelPositions getWheelPositions() {
		return new MecanumDriveWheelPositions(
				Encoders.frontLeft.getDistance(), Encoders.frontRight.getDistance(),
				Encoders.backLeft.getDistance(), Encoders.backRight.getDistance());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		mecanumDriveOdometry.update(Gyroscope.sensor.getRotation2d(), wheelPositions);

		// DEBUG: SmartDashboard entries
		SmartDashboard.putData("frontLeft", Motors.frontLeft);
		SmartDashboard.putData("frontRight", Motors.frontRight);
		SmartDashboard.putData("backLeft", Motors.backLeft);
		SmartDashboard.putData("backRight", Motors.backRight);
		SmartDashboard.putNumber("X Value", RobotContainer.driverOneController.getLeftX());
		SmartDashboard.putNumber("Y Value", RobotContainer.driverOneController.getLeftY());
		SmartDashboard.putNumber("Z Value", RobotContainer.driverOneController.getRightX());
	}
}
