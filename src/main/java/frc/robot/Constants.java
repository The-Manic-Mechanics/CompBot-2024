// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ComplexAuton;

// TODO: Complete reworking comments.
public final class Constants {
	public static class LimeLightMounting {
		/**
		 * How many degrees the limelight is mounted from perfectly vertical.
		 */
		public static final float MOUNT_ANGLE_DEGREES = 90;
		/**
		 * The amount of inches from the center of the LimeLight lens to the floor.
		 */
		public static final float LENS_HEIGHT_INCHES = 20;
	}

	public static class Motors {
		public static class Locations {
			public static class DriveTrain {
				/**
				 * The position of the front left wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final float FRONT_LEFT = 0;
				/**
				 * The position of the front right wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final float FRONT_RIGHT = 0;
				/**
				 * The position of the back left wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final float REAR_LEFT = 0;
				/**
				 * The position of the back right wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final float REAR_RIGHT = 0;
			}
		}
		public static class Ports {
			public static class DriveTrain {
				/**
				 * The CAN port of the front left motor.
				 */
				public static final byte FRONT_LEFT = 1;
				/**
				 * The CAN port of the front right motor.
				 */
				public static final byte FRONT_RIGHT = 2;
				/**
				 * The CAN port of the rear left motor.
				 */
				public static final byte REAR_LEFT = 3;
				/**
				 * The CAN port of the rear right motor.
				 */
				public static final byte REAR_RIGHT = 4;
			}
			public static class Intake {
				/**
				 * The CAN port of the lift motor on the intake mechanism.
				 */
				public static final byte LIFT = 8;
				/**
				 * The CAN port of the left motor on the intake mechanism.
				 */
				public static final byte LEFT= 9;
				/**
				 * The CAN port of the right motor on the intake mechanism. 
				 */
				public static final byte RIGHT = 10;
			}
			public static class Climber {
				/**
				 * The CAN port of the first motor on the climber mechanism.
				 */
				public static final byte ONE = 5;
				/**
				 * The CAN port of the second motor on the climber mechanism.
				 */
				public static final byte TWO = 6;
				/**
				 * The CAN port of the hook positioning motor on the climber mechanism.
				 */
				public static final byte HOOK_POSITIONER = 13;
			}
			public static class Shooter {
				/**
				 * The CAN port of the left motor on the shooter mechanism.
				 */
				public static final byte LEFT = 11;
				/**
				 * The CAN port of the right motor on the shooter mechanism.
				 */
				public static final byte RIGHT = 12;
			}
		}
	}

	public static class Shooter {
		/**
		 * The speed that the shooter spins at (As a percentage)
		 */
		public static final byte SPEED = 1;

		// TODO: Fill this in
		/**
		 * All the optimal positions the robot can shoot from.
		 */
		public static final Pose2d [] SHOOTING_POSITIONS = {};
	}

	public static class Encoders {
		public static class Ports {
			public static class Intake {
				/**
				 * The A channel port of the lift encoder.
				 */
				public static final byte LIFT_A = 0;
				/**
				 * The B channel port of the lift encoder.
				 */
				public static final byte LIFT_B = 1;
			}

			public static class Shooter {
				/**
				 * The CAN port of the left motor on the shooter mechanism.
				 */
				public static final byte LEFT = 11;
				/**
				 * The CAN port of the right motor on the shooter mechanism.
				 */
				public static final byte RIGHT = 12;
			}
		}
	

		public static class Intake {
			// TODO: Retake these values (New gear ratio on lift)
			/**
		 	* The lift's encoder's distance per pulse. (1 is not the real value)
		 	*/
			public static final short LIFT_DISTANCE_PER_PULSE = 1;
			/**
			 * The lowest point the intake can be driven to
			 */
			public static final short LOW_LIMIT = 690;
			/**
			 * The highest point the intake can be driven to
			 */
			public static final short HIGH_LIMIT = 40;
			/**
			 * The upper limit below which the intake will turn on
			 * (The lift goes below this limit and the intake motors turn on)
			 */
			public static final short ON_LIMIT = 486;
			/**
			 *  The upper limit above which the shooter turns on 
			 * (The lift goes above this threshold and the shooter motors activate)
			 */
			public static final short SHOOTER_ON_LIMIT = 267;
			/**
			 * The upper limit to the AMP scoring area.
			 */
			public static final short AMP_SCORING_POSITION_UPPER = 200;
			/**
			 * The lower limit to the AMP scoring area.
			 */
			public static final short AMP_SCORING_POSITION_LOWER = 300;
			/**
			 * The upper limit to the pickup position.
			 */
			public static final short PICKUP_POSITION_HIGHER = 600;
			/**
			 * The lower limit to the shooting position.
			 */
			public static final short SHOOTING_POSITION_LOWER = 100;
		}

	}
	
	public static class Climber {
		/**
		 * The speed the climber motors move at (aas a percentage).
		 */
		public static final float SPEED = 1;
		/**
		 * The average speed of the hook positioner.
		 */
		public static final float HOOK_POSITIONER_SPEED = 1;
	}

	public static class Intake {
		/**
		 * The speed of the actual intake motors.
		 */
		public static final float SPEED = .5f;
		/**
		 * The speed multiplier for the intake lift
		 */
		public static final float LIFT_SPEED_MULTIPLIER = .8f;
	}

	// TODO: Obtain this information:
	public static class Auton {
		/**
		 * The max forward speed of the robot in meters per second.
		 */
		public static final float MAX_SPEED = 7.613f;
		/**
		 * The backup initial coordinates for a no-autonomous path no-AprilTag setup.
		 */
		public static final Translation2d BACKUP_INITIAL_COORDINATES = new Translation2d(0, 0);
		/** 
		 * The prefered velocity of the robot in autonomous mode in meters per second.
		 */
		public static final float DRIVE_VEL = 0;
		/**
		 * The distance between the left and right wheels in meters.
		 */
		public static final float TRACK_WIDTH_METERS = 0.4699f;
		/**
		 * The distance the a wheel travels within one pulse on the encoders, in metres.
		 */
		public static final float DISTANCE_PER_PULSE = .00191f;
		/**
		 * The maximum acceleration of the robot in meters per second.
		 */
		public static final float MAX_ACCEL = 0;
		// TODO: Fix placeholder SPARKMAX_COUNTS_PER_REV value:
		/**
		 * The amount of ticks per revolution in the SPARK MAX motors.
		 */
		public static final short SPARKMAX_COUNTS_PER_REV = 250;
		/**
		 * The file paths to all PathWeaver paths in the deploy filesystem.
		 */
		public static final String[] ALL_PATHS_ORDER = {"output/Straight.wpilib.json"};
		/**
		 * A non-constant cache storage for loaded trajectories.
		 */
		public static Trajectory [] trajectories;
		/**
		 * Attempts to load a trajectory from each path described in ALL_PATHS_ORDER.
		 * @throws IOException If a trajectory failed to be opened or processed.
		 */
		public static void loadTrajectoriesFromPaths() {
			try {
				trajectories = ComplexAuton.loadPaths(Auton.ALL_PATHS_ORDER);
			} catch (IOException ex) {
				DriverStation.reportError("Failed to load a trajectory!", ex.getStackTrace());
			}
		}
		// TODO: Run a SysID characterization.
		public static class FeedForwardControllers {
			/**
			 * The static gain, determined by a SysID characterization.
			 */
			public static final float STATIC_GAIN = 0;
			/**
			 * The velocity gain, determined by a SysID characterization.
			 */
			public static final float VELOCITY_GAIN = 0;
			/**
			 * The acceleration gain, determined by a SysID characterization.
			 */
			public static final float ACCEL_GAIN = 0;
		}
	}
	public static class PIDControllers {
		public static class Holonomic {
			/**
			 * TODO: Change accordingly.
			 * The P constant for the holonomicController's X correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final float XCONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the holonomicController's X correction.
			 * PID loop.
			 */
			public static final float XCONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the holonomicController's X correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final float XCONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the holonomicController's Y correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final float YCONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the holonomicController's Y correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final float YCONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the holonomicController's Y correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final float YCONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the holonomicController's rotation correction PID loop.
			 * Units are in degrees.
			 */
			public static final float THETACONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the holonomicController's rotation correction PID loop.
			 * Units are in degrees.
			 */
			public static final float THETACONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the holonomicController's rotation correction PID loop.
			 * Units are in degrees.
			 */
			public static final float THETACONTROLLER_D = 0;
		}

		public static class WheelVelocities {
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * frontLeft motor.
			 */
			public static final float FL_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * frontLeft motor.
			 */
			public static final float FL_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * frontLeft motor.
			 */
			public static final float FL_CONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * frontRight motor.
			 */
			public static final float FR_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * frontRight motor.
			 */
			public static final float FR_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * frontRight motor.
			 */
			public static final float FR_CONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * rearLeft motor.
			 */
			public static final float RL_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * rearLeft motor.
			 */
			public static final float RL_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * rearLeft motor.
			 */
			public static final float RL_CONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * rearRight motor.
			 */
			public static final float RR_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * rearRight motor.
			 */
			public static final float RR_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * rearRight motor.
			 */
			public static final float RR_CONTROLLER_D = 0;
		}
	}
}