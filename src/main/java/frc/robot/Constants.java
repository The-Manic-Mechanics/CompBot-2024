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

	public static class DriveTrain {
		public static class MotorPorts {
			// TODO: Fill in port traffic information once we have it.
			/**
			 * Front Left CAN Port
			 */
			public static final int
			FRONT_LEFT = 0,
			/**
			 * Front Right CAN Port
			 */
			FRONT_RIGHT = 0,
			/**
			 * Back Left CAN Port
			 */
			BACK_LEFT = 0,
			/**
			 * Back Right CAN Port
			 */
			BACK_RIGHT = 0;
		}

		public static class EncoderPorts {
			// TODO: Fill in port traffic information once we have it.
			/**
			 * Front Left Encoder Channel A
			 */
			public static final int
			FRONT_LEFT_A = 0,
			/**
			 * Front Left Encoder Channel B
			 */
			FRONT_LEFT_B = 0,
			/**
			 * Front Right Encoder Channel A
			 */
			FRONT_RIGHT_A = 0,
			/**
			 * Front Right Encoder Channel B
			 */
			FRONT_RIGHT_B = 0,
			/**
			 * Back Left Encoder Channel A
			 */
			BACK_LEFT_A = 0,
			/**
			 * Back Left Encoder Channel B
			 */
			BACK_LEFT_B = 0,
			/**
			 * Back Right Encoder Channel A
			 */
			BACK_RIGHT_A = 0,
			/**
			 * Back Right Encoder Channel B
			 */
			BACK_RIGHT_B = 0;
		}

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
	}
}