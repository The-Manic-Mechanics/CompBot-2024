// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
	public static class Controllers {
		/**
		 * The port which the first driver's controller is connected to.
		 */
		public static final byte DRIVERONE_PORT = 0;
		/**
		 * The port which the second driver's controller is connected to.
		 */
		public static final byte DRIVERTWO_PORT = 1;

		public static class Sax {
			public static final byte PORT = 2;
			public static class ButtonPorts {
				public static final byte ORANGE = 1;
				public static final byte RED = 2;
				public static final byte BLUE = 3;
				public static final byte GREEN = 4;
				public static final byte SALMON = 5;
				public static final byte YELLOW = 6;
				public static final byte PINK = 7;
				public static final byte PURPLE = 8;
				public static final byte JOYSTICK = 13;
			}
			public static class AxisPort {
				/**
				 * X is towards the bell for positive and away for negative
				 * Y is towards the buttons for positive and away for negative
				 */
				public static final byte X = 0;
				public static final byte Y = 1;
			}
		}
	}

	public static class LimeLightMounting {
		/**
		 * How many degrees the limelight is mounted from perfectly vertical.
		 */
		public static final double limelightMountAngleDegrees = 90;
		/**
		 * The amount of inches from the center of the LimeLight lens to the floor.
		 */
		public static final double lensHeightInches = 20;
	}

	public static class Motors {
		public static class Locations {
			public static class DriveTrain {
				/**
				 * The position of the front left wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final double FRONT_LEFT = 0;
				/**
				 * The position of the front right wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final double FRONT_RIGHT = 0;
				/**
				 * The position of the back left wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final double BACK_LEFT = 0;
				/**
				 * The position of the back right wheel relative to the center of the robot, in
				 * meters.
				 */
				public static final double BACK_RIGHT = 0;
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
				 * The CAN port of the back left motor.
				 */
				public static final byte BACK_LEFT = 3;
				/**
				 * The CAN port of the back right motor.
				 */
				public static final byte BACK_RIGHT = 4;
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
	}

	public static class Encoders {
		public static class Ports {
			public static class DriveTrain {
				/**
				 * The A channel port of the front left encoder.
				 */
				public static final byte FRONT_LEFT_A = 30;
				/**
				 * The B channel port of the front left encoder.
				 */
				public static final byte FRONT_LEFT_B = 40;
				/**
				 * The A channel port of the front right encoder.
				 */
				public static final byte FRONT_RIGHT_A = 50;
				/**
				 * The B channel port of the front right encoder.
				 */
				public static final byte FRONT_RIGHT_B = 60;
				/**
				 * The A channel port of the back left encoder.
				 */
				public static final byte BACK_LEFT_A = 70;
				/**
				 * The B channel port of the back left encoder.
				 */
				public static final byte BACK_LEFT_B = 80;
				/**
				 * The A channel port of the back right encoder.
				 */
				public static final byte BACK_RIGHT_A = 90;
				/**
				 * The B channel port of the back right encoder.
				 */
				public static final byte BACK_RIGHT_B = 100;
			}
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
	}

	public static class Intake {
		// TODO: Rework comments
		// TODO: Retake these values (New gear ratio on lift)
		/**
		 * The intake lift encoder's distance per pulse
		 */
		public static final short LIFT_DISTANCE_PER_PULSE = 1;
		/**
		 * The lowest point the intake can be driven to
		 */
		public static final short LOWER_LIMIT = 690;
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
		 * The upper limit to the amp scoring area
		 */
		public static final short AMP_SCORING_POSITION_UPPER = 200;
		/**
		 * The lower limit to the amp scoring area
		 */
		public static final short AMP_SCORING_POSITION_LOWER = 300;
		/**
		 * The upper limit to the pickup position
		 */
		public static final short PICKUP_POSITION_HIGHER = 600;
		/**
		 * The lower limit to the shooting position
		 */
		public static final short SHOOTING_POSITION_LOWER = 100;
		/**
		 * The speed multiplier for the intake lift
		 */
		public static final double LIFT_SPEED_MULTIPLIER = .8;
		/**
		 * The speed of the actual intake motors
		 */
		public static final double SPEED = 1;
	}
	
	public static class Climber {
		// TODO: Rework comments
		public static final double SPEED = 1;
		/**
		 * The average speed of the hook positioner
		 */
		public static final double HOOK_POSITIONER_SPEED = 1;
	}

	public static class Auton {
		/**
		 * The max forward speed of the robot in meters per second.
		 */
		public static final double MAX_SPEED = 0;
		/**
		 * The prefered velocity of the robot in autonomous mode in meters per second.
		 */
		public static final double DRIVE_VEL = 0;
		/**
		 * The distance between the left and right wheels in meters.
		 */
		public static final double TRACK_WIDTH_METERS = 0;
		/**
		 * The distance between pulses of the encoders in feet.
		 */
		public static final double DISTANCE_PER_PULSE = 0;
		/**
		 * The maximum acceleration of the robot in meters per second.
		 */
		public static final double MAX_ACCEL = 0;

		// TODO: Factor in the wheel diameters and gear ratios
		public static final byte SPARKMAX_COUNTS_PER_REV = 42;

		public static class FeedForwardControllers {
			/**
			 * The static gain, determined by a SysID characterization.
			 */
			public static final double STATIC_GAIN = 0;
			/**
			 * The velocity gain, determined by a SysID characterization.
			 */
			public static final double VELOCITY_GAIN = 0;
			/**
			 * The acceleration gain, determined by a SysID characterization.
			 */
			public static final double ACCEL_GAIN = 0;
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
			public static final double XCONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the holonomicController's X correction.
			 * PID loop.
			 */
			public static final double XCONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the holonomicController's X correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final double XCONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the holonomicController's Y correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final double YCONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the holonomicController's Y correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final double YCONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the holonomicController's Y correction.
			 * PID loop.
			 * Position units are field relative.
			 */
			public static final double YCONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the holonomicController's rotation correction PID loop.
			 * Units are in degrees.
			 */
			public static final double THETACONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the holonomicController's rotation correction PID loop.
			 * Units are in degrees.
			 */
			public static final double THETACONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the holonomicController's rotation correction PID loop.
			 * Units are in degrees.
			 */
			public static final double THETACONTROLLER_D = 0;
		}

		public static class WheelVelocities {
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * frontLeft motor.
			 */
			public static final double FL_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * frontLeft motor.
			 */
			public static final double FL_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * frontLeft motor.
			 */
			public static final double FL_CONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * frontRight motor.
			 */
			public static final double FR_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * frontRight motor.
			 */
			public static final double FR_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * frontRight motor.
			 */
			public static final double FR_CONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * rearLeft motor.
			 */
			public static final double RL_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * rearLeft motor.
			 */
			public static final double RL_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * rearLeft motor.
			 */
			public static final double RL_CONTROLLER_D = 0;
			/**
			 * TODO: Change accordingly.
			 * The P constant for the individual motor velocity PIDController of the
			 * rearRight motor.
			 */
			public static final double RR_CONTROLLER_P = 0;
			/**
			 * TODO: Change accordingly.
			 * The I constant for the individual motor velocity PIDController of the
			 * rearRight motor.
			 */
			public static final double RR_CONTROLLER_I = 0;
			/**
			 * TODO: Change accordingly.
			 * The D constant for the individual motor velocity PIDController of the
			 * rearRight motor.
			 */
			public static final double RR_CONTROLLER_D = 0;
		}
	}
}