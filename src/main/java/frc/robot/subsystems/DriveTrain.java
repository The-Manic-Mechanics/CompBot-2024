// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.Auton;
import frc.robot.commands.DriveAuton;
import java.util.function.Supplier;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

public final class DriveTrain extends SubsystemBase {
	public static MecanumDrive mecanum;

	public static class Motors {
		public static CANSparkMax frontLeft;
		public static CANSparkMax frontRight;
		public static CANSparkMax rearLeft;
		public static CANSparkMax rearRight;
	}

	public static class Encoders {
		public static RelativeEncoder frontLeft;
		public static RelativeEncoder frontRight;
		public static RelativeEncoder rearLeft;
		public static RelativeEncoder rearRight;
	}

	public static class Odometry {
		/**
		 * Used for keeping track of the robot's position while it drives.
		 */
		public static MecanumDriveOdometry mecanumDriveOdometry;

		public static SendableChooser<Object> autonRoutineChooser = new SendableChooser<Object>();
		public static SendableChooser<Pose2d> autonStartPositionChooser = new SendableChooser<Pose2d>();

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
					DriveTrain.Encoders.frontLeft.getPosition(),
					DriveTrain.Encoders.frontRight.getPosition(),
					DriveTrain.Encoders.rearLeft.getPosition(),
					DriveTrain.Encoders.rearRight.getPosition());
		}

		/**
		 * Supplies the DriveTrain wheel speeds.
		 * 
		 * @return A filled-out MecanumDriveWheelSpeeds instance.
		 */
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
		Motors.frontLeft = new CANSparkMax(Constants.Motors.Ports.DriveTrain.FRONT_LEFT, MotorType.kBrushless);
		Motors.frontRight = new CANSparkMax(Constants.Motors.Ports.DriveTrain.FRONT_RIGHT, MotorType.kBrushless);
		Motors.rearLeft = new CANSparkMax(Constants.Motors.Ports.DriveTrain.REAR_LEFT, MotorType.kBrushless);
		Motors.rearRight = new CANSparkMax(Constants.Motors.Ports.DriveTrain.REAR_RIGHT, MotorType.kBrushless);

		Encoders.frontLeft = Motors.frontLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
		Encoders.frontRight = Motors.frontRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
		Encoders.rearLeft = Motors.rearLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
		Encoders.rearRight = Motors.rearRight.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

		Motors.frontLeft.setInverted(true);
		Motors.rearLeft.setInverted(true);

		mecanum = new MecanumDrive(
				Motors.frontLeft,
				Motors.rearLeft,
				Motors.frontRight,
				Motors.rearRight);

		Kinematics.mecanumDriveKinematics = new MecanumDriveKinematics(
				Constants.Motors.Locations.DriveTrain.FRONT_LEFT,
				Constants.Motors.Locations.DriveTrain.FRONT_RIGHT,
				Constants.Motors.Locations.DriveTrain.REAR_LEFT,
				Constants.Motors.Locations.DriveTrain.REAR_RIGHT
		);

		Kinematics.wheelPositions = new MecanumDriveWheelPositions(
				Encoders.frontLeft.getPosition(),
				Encoders.frontRight.getPosition(),
				Encoders.rearLeft.getPosition(),
				Encoders.rearRight.getPosition());

		Constants.Auton.loadTrajectoriesFromPaths();

		// TODO: Where are these locations really (and should null be default? handle this?)
		// Odometry.autonStartPositionChooser.setDefaultOption("Center", new Pose2d());
		// Odometry.autonStartPositionChooser.addOption("Left", new Pose2d());
		// Odometry.autonStartPositionChooser.addOption("Right", new Pose2d());

		// SmartDashboard.putData("Start Position Chooser", Odometry.autonStartPositionChooser);

		Odometry.autonRoutineChooser.setDefaultOption("None", null);
		Odometry.autonRoutineChooser.addOption("Straight Auton Center", new DriveAuton(this, new Gyroscope(), 24, -.1, 0, 0));
		Odometry.autonRoutineChooser.addOption("Trajectory One", Auton.trajectories[0]);
		Odometry.autonRoutineChooser.addOption("Straight Auton Wall", new DriveAuton(this, new Gyroscope(), 48, -.1, 0, 0));

		
		SmartDashboard.putData("Auton Path Chooser", Odometry.autonRoutineChooser);

		Odometry.mecanumDriveOdometry = new MecanumDriveOdometry(
				Kinematics.mecanumDriveKinematics,
				Gyroscope.sensor.getRotation2d(),	
				Kinematics.wheelPositions,
				Odometry.autonRoutineChooser.getSelected() == null ? 
					LimeLight.tagID == 0 ?
						new Pose2d(Constants.Auton.BACKUP_INITIAL_COORDINATES, Gyroscope.sensor.getRotation2d())
						:
						LimeLight.getBotPose2d()
					:
					(Odometry.autonRoutineChooser.getSelected() instanceof Trajectory) ?
						((Trajectory)Odometry.autonRoutineChooser.getSelected()).getInitialPose()
						:
						// TODO: Get data from position chooser rather than from backup coordinates ^^^
						new Pose2d(Constants.Auton.BACKUP_INITIAL_COORDINATES, Gyroscope.sensor.getRotation2d())
		);
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
	}
}
