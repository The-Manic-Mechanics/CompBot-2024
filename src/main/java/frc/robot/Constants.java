// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class Controllers {
		/**
		 * TODO: Change accordingly.
		 * The port which the first driver's controller is connected to.
		 */
		public static final int
            DRIVERONE_PORT = 0,
            /**
			 * TODO: Change accordingly.
             * The port which the second driver's controller is connected to.
             */
            DRIVERTWO_PORT = 1;
		public static class Sax {
			/**
			 * The port which the saxophone controller is connected to
			 */
			public static final int
				SAX_PORT = 2;
			public static class ButtonsPort {
				public static final int       
					ORANGE = 0,
					RED = 1,
					BLUE = 2,
					GREEN = 3,
					SALMON = 4,
					YELLOW = 5,
					PINK = 6,
					PURPLE = 7,
					JOYSTICK = 12;
			}

			public static class AxisPort {
				/**
				 * X is towards the bell for positive and away for negative
				 * Y is towards the buttons for positive and away for negative
				 */
				public static final int
					X = 0,
					Y = 1;
			}
		}
	}
	
	public static class LimeLightOffsets {
		/**
		 * TODO: Change accordingly.
         * How far offset the robot is from being collinear with the AprilTag (Vertical).
         */
	    public static final double
	         /**
			  * TODO: Change accordingly.
              * How many degrees the limelight is mounted from perfectly vertical.
              */
            limelightMountAngleDegrees = 90,
			
            /**
			 * TODO: Change accordingly.
             * The amount of inches from the center of the LimeLight lens to the floor.
             */
			lensHeightInches = 20;
	}

	public static class AprilTagCoords {
		/**
		 * FIXME: What is this?
		 * Direction: 1 (or !(2 or 3)) is Forward, 2 is Left, 3 is Right
		 */
		public static Translation2d TranslationFromAprilTagCoordinate(double in_id, int direction) {
			int id = (int) in_id;
			if (direction == 2) // Left
				// TODO: Change accordingly. (the added constant)
				return new Translation2d(MAP[id][0], MAP[id][1] + 13.75);
			else if (direction == 3) // Right
				// TODO: Change accordingly. (the subtracted constant)
				return new Translation2d(MAP[id][0], MAP[id][1] - 13.75);
			else
				return new Translation2d(MAP[id][0], MAP[id][1]);
		}

		/**
		 * TODO: Change accordingly.
		 * X, Y, Z, Heading (In that order)
		 */
		public static final double[][] MAP = {
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				},
				{
						0.0, 0.0, 0.0, 0.0
				}
		};
	}

	public static class Motors {
		public static class Ports {
			public static class DriveTrain {
				// TODO: Fill in port traffic information once we have it.
				/**
				 * Front Left CAN Port
				 */
				public static final int
					FRONT_LEFT = 1,
					/**
					 * Front Right CAN Port
					 */
					FRONT_RIGHT = 2,
					/**
					 * Back Left CAN Port
					 */
					BACK_LEFT = 3,
					/**
					 * Back Right CAN Port
					 */
					BACK_RIGHT = 4;
			}

			public static class Intake {
				public static int
					/**
					 * Intake lift motor CAN port
					 */
					LIFT = 8,
					/**
					 * Intake left motor CAN port
					 */
					LEFT= 9,
					/**
					 * Intake right motor CAN port 
					 */
					RIGHT = 10;
			}

			public static class Climber {
				public static int
					/**
					 * First climber motor CAN port
					 */
					ONE = 5,
					/**
					 * Second climber motor CAN port
					 */
					TWO = 6,
					/**
					 * The hook positioner motor CAN port
					 */
					HOOK_POSITIONER = 13;
			}

			public static class Shooter {
				public static int
					/**
					 * Shooter left motor CAN port
					 */
					LEFT = 11,
					/**
					 * Shooter right motor CAN port
					 */
					RIGHT = 12;
			}
		}

	}

	public static class Encoders {
		public static class Ports {
			public static class DriveTrain {
				// TODO: Fill in port traffic information once we have it.
				/**
				 * Front Left Encoder Channel A
				 */
				public static final int
					FRONT_LEFT_A = 30,
					/**
					 * Front Left Encoder Channel B
					 */
					FRONT_LEFT_B = 40,
					/**
					 * Front Right Encoder Channel A
					 */
					FRONT_RIGHT_A = 50,
					/**
					 * Front Right Encoder Channel B
					 */
					FRONT_RIGHT_B = 60,
					/**
					 * Back Left Encoder Channel A
					 */
					BACK_LEFT_A = 70,
					/**
					 * Back Left Encoder Channel B
					 */
					BACK_LEFT_B = 80,
					/**
					 * Back Right Encoder Channel A
					 */
					BACK_RIGHT_A = 90,
					/**
					 * Back Right Encoder Channel B
					 */
					BACK_RIGHT_B = 100;
			}

			public static class Intake {
				public static int
					/**
					 * Lift encoder channel A
					 */
					LIFT_A = 0,
					/**
					 * Lift encoder channel B
					 */
					LIFT_B = 1;
			}
		}

		public static class Intake {
				public static int
					/**
					 * The intake lift encoder's distance per pulse
					 */
					LIFT_DISTANCE_PER_PULSE = 1,
		
					/**
					 * The lowest point the intake can be driven to
					 */
					LOWER_LIMIT = 690,
		
					/**
					 * The highest point the intake can be driven to
					 */
					HIGH_LIMIT = 40,
					/**
					 * The upper limit below which the intake will turn on
					 * (The lift goes below this limit and the intake motors turn on)
					 */
					ON_LIMIT = 486,
					/**
					 *  The upper limit above which the shooter turns on 
					 * (The lift goes above this threshold and the shooter motors activate)
					 */
					SHOOTER_ON_LIMIT = 267,
					/**
					 * The upper limit to the amp scoring area
					 */
					AMP_SCORING_POSITION_UPPER = 200,
					/**
					 * The lower limit to the amp scoring area
					 */
					AMP_SCORING_POSITION_LOWER = 300;

					/**
					 * The upper limit to the pickup position
					 */
					public static final int PICKUP_POSITION_HIGHER = 600;

					/**
					 * The lower limit to the shooting position
					 */
					public static final int SHOOTING_POSITION_LOWER = 100;
		}
	}

	public static class DriveTrain {
		public static class MotorLocations {
			/**
			 * TODO: Change accordingly.
			 * Where the front left wheel is located relative to the center of the robot.
			 */
			public static final double
				FRONT_LEFT = 0.0,
				/**
				 * TODO: Change accordingly.
				 * Where the front right wheel is located relative to the center of the robot.
				 */
				FRONT_RIGHT = 0.0,
				/**
				 * TODO: Change accordingly.
				 * Where the back left  wheel is located relative to the center of the robot in metres.
				 */
				BACK_LEFT = 0.0,
				/**
				 * TODO: Change accordingly.
				 * Where the back right wheel is located relative to the center of the robot in metres.
				 */
				BACK_RIGHT = 0.0;
		}			
	}

	public static class Intake {
		public static double
			/**
			 * The speed multiplier for the intake lift
			 */
			LIFT_SPEED_MULTIPLIER = .8,
			/**
			 * The speed of the actual intake motors
			 */
			SPEED = 1;
	}

	public static class Climber {
		public static double SPEED = 1;
		/**
		 * The average speed of the hook positioner
		 */
		public static double HOOK_POSITIONER_SPEED = 1;
	}

	public static class Auton {
		/**
		 * TODO: Change accordingly.
		 * The max forward speed of the robot in meters per second.
		 */
		public static final double
		MAX_METRES_PER_SEC = 0, 
		/**
		 * TODO: Change accordingly.
		 * The max acceleration of the robot in meters per second.
		 */
		MAX_ACCEL = 0,
		/**
		 * TODO: Change accordingly.
		 * The prefered velocity of the robot in autonomous mode in meters per second.
		 */
		DRIVE_VEL = 0,
		/**
		 * TODO: Change accordingly.
		 * The distance between the left and right wheels.
		 */
		TRACK_WIDTH_METRES = 0,
		/**
		 * TODO: Change accordingly.
		 * The distance (In feet) per each pulse on the encoder
		 */
		DISTANCE_PER_PULSE = 0;

		public static final java.util.HashMap<String, Command> EVENT_MAP = new java.util.HashMap<>();
		
		// TODO: Factor in the wheel diameters and gear ratios
		public static final int SPARKMAX_COUNTS_PER_REV = 42;
	}
	public static class Shooter {
		public static double
			/**
			 * The speed that the shooter spins at (As a percentage)
			 */
			SPEED = 1;
	}
}
