// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Auton;
import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.can.*;

public final class DriveTrain extends SubsystemBase {
	public static MecanumDrive mecanum;

	public static class Motors {
		public static WPI_VictorSPX frontLeft, frontRight, rearLeft, rearRight;
	}

	public static class Encoders {
		public static Encoder frontLeft, frontRight, rearLeft, rearRight;
	}

	public static class Odometry {
		/**
		 * Used for keeping track of the robot's position while it drives.
		 */
		public static MecanumDriveOdometry mecanumDriveOdometry;

		/**
		 * Sets the driveOdometry values to those supplied in the Pose2d.
		 * 
		 * @param pose The pose to set the driveOdometry values to.
		 */
		public static void resetDriveOdometry(Pose2d pose) {
			mecanumDriveOdometry.resetPosition(
					Gyroscope.sensor.getRotation2d(),
					Kinematics.getWheelPositions(),
					pose);
		}

	}

	public static class Kinematics {
		public static MecanumDriveKinematics mecanumDriveKinematics;
		private static MecanumDriveWheelPositions wheelPositions;
		public static MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds;

		public static Supplier<ChassisSpeeds> getMecanumChassisSpeeds() {
			return (Supplier<ChassisSpeeds>) () -> mecanumDriveKinematics.toChassisSpeeds(mecanumDriveWheelSpeeds);
		}

		/**
		 * Gets the current wheel positions.
		 *
		 * @return A filled-out MecanumDriveWheelPositions instance.
		 */
		public static MecanumDriveWheelPositions getWheelPositions() {
			return new MecanumDriveWheelPositions(
					DriveTrain.Encoders.frontLeft.getDistance(), DriveTrain.Encoders.frontRight.getDistance(),
					DriveTrain.Encoders.rearLeft.getDistance(), DriveTrain.Encoders.rearRight.getDistance());
		}

		/**
		 * Supplies the DriveTrain wheel speeds.
		 * 
		 * @return A filled-out MecanumDriveWheelSpeeds instance.
		 */
		// FIXME: Not sure if this should be negative or not
		public static Supplier<MecanumDriveWheelSpeeds> getWheelSpeeds() {
			Kinematics.mecanumDriveWheelSpeeds = new MecanumDriveWheelSpeeds(
					Motors.frontLeft.get() * Auton.MAX_SPEED,
					Motors.frontRight.get() * Auton.MAX_SPEED,
					Motors.rearLeft.get() * Auton.MAX_SPEED,
					Motors.rearRight.get() * Auton.MAX_SPEED);
			return (Supplier<MecanumDriveWheelSpeeds>) () -> Kinematics.mecanumDriveWheelSpeeds;
		}

		/**
		 * Sets voltages to each of the motors.
		 * 
		 * @param inVolts The voltages to set each motor with.
		 */
		public static void driveVolts(MecanumDriveMotorVoltages inVolts) {
			Motors.frontLeft.setVoltage(inVolts.frontLeftVoltage);
			Motors.frontRight.setVoltage(inVolts.frontRightVoltage);
			Motors.rearLeft.setVoltage(inVolts.rearLeftVoltage);
			Motors.rearRight.setVoltage(inVolts.rearRightVoltage);
		}
	}

	public DriveTrain() {
		DriveTrain.Motors.frontLeft = new WPI_VictorSPX(Constants.Motors.Ports.DriveTrain.FRONT_LEFT);
		Motors.frontRight = new WPI_VictorSPX(Constants.Motors.Ports.DriveTrain.FRONT_RIGHT);
		Motors.rearLeft = new WPI_VictorSPX(Constants.Motors.Ports.DriveTrain.BACK_LEFT);
		Motors.rearRight = new WPI_VictorSPX(Constants.Motors.Ports.DriveTrain.BACK_RIGHT);
		Motors.frontLeft.setInverted(true);
		Motors.rearLeft.setInverted(true);

		mecanum = new MecanumDrive(
				Motors.frontLeft,
				Motors.rearLeft,
				Motors.frontRight,
				Motors.rearRight);

		Encoders.frontLeft = new Encoder(
				Constants.Encoders.Ports.DriveTrain.FRONT_LEFT_A,
				Constants.Encoders.Ports.DriveTrain.FRONT_LEFT_B);
		Encoders.frontRight = new Encoder(
				Constants.Encoders.Ports.DriveTrain.FRONT_RIGHT_A,
				Constants.Encoders.Ports.DriveTrain.FRONT_RIGHT_B);
		Encoders.rearLeft = new Encoder(
				Constants.Encoders.Ports.DriveTrain.BACK_LEFT_A,
				Constants.Encoders.Ports.DriveTrain.BACK_LEFT_B);
		Encoders.rearRight = new Encoder(
				Constants.Encoders.Ports.DriveTrain.BACK_RIGHT_A,
				Constants.Encoders.Ports.DriveTrain.BACK_RIGHT_B);
		Encoders.frontLeft.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.frontRight.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.rearLeft.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);
		Encoders.rearRight.setDistancePerPulse(Auton.DISTANCE_PER_PULSE);

		Kinematics.mecanumDriveKinematics = new MecanumDriveKinematics(
				new Translation2d(Constants.Motors.Locations.DriveTrain.FRONT_LEFT,
						Constants.Motors.Locations.DriveTrain.FRONT_LEFT),
				new Translation2d(Constants.Motors.Locations.DriveTrain.FRONT_RIGHT,
						-1 * Constants.Motors.Locations.DriveTrain.FRONT_RIGHT),
				new Translation2d(-1 * Constants.Motors.Locations.DriveTrain.BACK_LEFT,
						Constants.Motors.Locations.DriveTrain.BACK_LEFT),
				new Translation2d(-1 * Constants.Motors.Locations.DriveTrain.BACK_RIGHT,
						-1 * Constants.Motors.Locations.DriveTrain.BACK_RIGHT));

		Kinematics.wheelPositions = new MecanumDriveWheelPositions(
				Encoders.frontLeft.getDistance(),
				Encoders.frontRight.getDistance(),
				Encoders.rearLeft.getDistance(),
				Encoders.rearRight.getDistance());

		Odometry.mecanumDriveOdometry = new MecanumDriveOdometry(
				Kinematics.mecanumDriveKinematics,
				Gyroscope.sensor.getRotation2d(),
				Kinematics.wheelPositions,
				RobotContainer.initPose);
	}

	@Override
	public void periodic() {
		Kinematics.wheelPositions = Kinematics.getWheelPositions();

		Odometry.mecanumDriveOdometry.update(
				Gyroscope.sensor.getRotation2d(),
				Kinematics.wheelPositions);

		if (LimeLight.tagID != 0)
			Odometry.mecanumDriveOdometry.resetPosition(
					Gyroscope.sensor.getRotation2d(),
					Kinematics.wheelPositions,
					LimeLight.getBotPose2d());

		SmartDashboard.putNumber("X Value", RobotContainer.driverOneController.getLeftX());
		SmartDashboard.putNumber("Y Value", RobotContainer.driverOneController.getLeftY());
		SmartDashboard.putNumber("Z Value", RobotContainer.driverOneController.getRightX());
	}
}
