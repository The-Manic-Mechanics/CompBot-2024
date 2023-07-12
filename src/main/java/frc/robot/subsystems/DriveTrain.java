// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
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
		public static WPI_VictorSPX frontLeft, frontRight, backLeft, backRight;

        /**
        * Sets the speeds of all the drive motors to inSpeeds
         * @param inSpeeds The speed to set the drive motors to
        */
		public static void setSpeeds(MecanumDriveWheelSpeeds inSpeeds) {
			assert inSpeeds != null;
			Motors.frontLeft.set(inSpeeds.frontLeftMetersPerSecond);
			Motors.frontRight.set(inSpeeds.frontRightMetersPerSecond);
			Motors.backLeft.set(inSpeeds.rearLeftMetersPerSecond);
			Motors.backRight.set(inSpeeds.rearRightMetersPerSecond);
		}

        /**
        * Sets the voltages of all the drive motors to inVolts
         * @param inVolts The voltage to set the drive motors to
        */
		public static void setVolts(MecanumDriveMotorVoltages inVolts) {
			assert inVolts != null;
			Motors.frontLeft.setVoltage(inVolts.frontLeftVoltage);
			Motors.frontRight.setVoltage(inVolts.frontRightVoltage);
			Motors.backLeft.setVoltage(inVolts.rearLeftVoltage);
			Motors.backRight.setVoltage(inVolts.rearRightVoltage);
		}
	}

	public static class Encoders {
		public static Encoder frontLeft, frontRight, backLeft, backRight;
	}

	public DriveTrain() {
		Motors.frontLeft = new WPI_VictorSPX(MotorPorts.FRONT_LEFT);
		Motors.frontRight = new WPI_VictorSPX(MotorPorts.FRONT_RIGHT);
		Motors.backLeft = new WPI_VictorSPX(MotorPorts.BACK_LEFT);
		Motors.backRight = new WPI_VictorSPX(MotorPorts.BACK_RIGHT);

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
		// frontRight.setInverted(true);
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

		Pose2d initPose = new Pose2d(currentPose[1], currentPose[2], NavX.sensor.getRotation2d());

		mecanumDriveOdometry = new MecanumDriveOdometry(mecanumDriveKinematics, NavX.sensor.getRotation2d(), wheelPositions,
				initPose);
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

	/**
	 * Generates a movement path for the robot, from point A to B <i>(transPx)</i>
	 * @param maxVel The maximum velocity to use while driving this path
	 * @param maxAccel The maximum acceleration to use while driving this path
	 * @param transP1 The initial pose of the robot before path is driven as a <b>Translation2d</b>
	 * @param rotHead1 The initial heading of the robot before the path is driven
	 * @param rotHolo1 The initial holonomic rotation of the robot before the path is driven
	 * @param transP2 The destination pose of the robot as a <b>Translation2d</b>
	 * @param rotHead2 The destination heading of the robot
	 * @param rotHolo2 The destination holonomic rotation of the robot
	 *
	 * @return  The generated path. <i>(type PathPlannerTrajectory)</i>
	 * @see #followTrajectoryCommand(PathPlannerTrajectory, boolean) to create a command to follow this path.
	 */
	public static PathPlannerTrajectory genPath (
		    double maxVel,
			double maxAccel,
			Translation2d transP1,
			double rotHead1,
		    double rotHolo1,
			Translation2d transP2,
			double rotHead2,
			double rotHolo2
	) {
		return PathPlanner.generatePath(
				new PathConstraints(maxVel, maxAccel),
				new PathPoint // position, heading, holonomic rotation
                        (
                                transP1,
                                Rotation2d.fromDegrees(rotHead1),
                                Rotation2d.fromDegrees(rotHolo1)
                        ),
				new PathPoint // position, heading, holonomic rotation
                        (
                                transP2,
                                Rotation2d.fromDegrees(rotHead2),
                                Rotation2d.fromDegrees(rotHolo2)
                        )
		);
	}

    /**
    * Resets the mecanumDriveOdometry using the resetPosition method
     * @param odoResetPose The <b>Pose2d</b> representing where the robot currently is
    */

	private static void resetOdometry(Pose2d odoResetPose) {
		mecanumDriveOdometry.resetPosition(NavX.sensor.getRotation2d(), getWheelPositions(), odoResetPose);
	}

    /**
    * Generates a command that follows the inputed trajectory
     * @param trajectory The PathPlannerTrajectory to be followed
     * @return A command that follows the inputed trajectory
    */
	public static Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {

		return new SequentialCommandGroup(
				new InstantCommand(() -> {
					// Reset odometry for the first path you run during auto
					if (isFirstPath) {
						mecanumDriveOdometry.resetPosition(null, wheelPositions, mecanumDriveOdometry.getPoseMeters());
					}
				}),
				new PPMecanumControllerCommand(
						trajectory,
						mecanumDriveOdometry::getPoseMeters, // Pose supplier
						mecanumDriveKinematics, // MecanumDriveKinematics
						new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use
						// feedforwards.
						new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
						new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
						// only use feedforwards.
						Auton.MAX_METRES_PER_SEC, // Max wheel velocity meters per second
						Motors::setSpeeds, // MecanumDriveWheelSpeeds consumer
						true, // Should the path be automatically mirrored depending on alliance color.
						// Optional, defaults to true
						new DriveTrain() // Requires this drive subsystem
				)
		);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		mecanumDriveOdometry.update(NavX.sensor.getRotation2d(), wheelPositions);

		SmartDashboard.putData("frontLeft", Motors.frontLeft);
		SmartDashboard.putData("frontRight", Motors.frontRight);
		SmartDashboard.putData("backLeft", Motors.backLeft);
		SmartDashboard.putData("backRight", Motors.backRight);

		// Putting Controller Left and Right Stick Values
		SmartDashboard.putNumber("X Value", RobotContainer.driverMainController.getLeftX());
		SmartDashboard.putNumber("Y Value", RobotContainer.driverMainController.getLeftY());
		SmartDashboard.putNumber("Z Value", RobotContainer.driverMainController.getRightX());
	}
}
