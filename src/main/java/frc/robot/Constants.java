// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight;

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
		 * The port which the first drivers controller is on
		 */
		public static final int
            DRIVERONE_PORT = 0,
            /**
            * The port which the second drivers controller is on
            */
            DRIVERTWO_PORT = 1;
	}
	
	public static class LimeLightOffsets {
		/**
        * How far offset the robot is from collinear with the april tag (Vertical)
        */
	    public static final double
	        aprilTagOffsetAngle = LimeLight.tY.getDouble(0.0),
	        
	         /**
             * How many degrees the limelight is mounted from perfectly vertical
             */
            limelightMountAngleDegrees = 90,

            /**
            * The angle to the apriltag to the robot in degrees
            */
            angleToAprilTagDegrees = limelightMountAngleDegrees + aprilTagOffsetAngle,
            
            /**
            * The angle to the apriltag in radians
            */
            angleToAprilTagRadians = angleToAprilTagDegrees * (3.14159 / 180.0),

            /**
            * Distance from the center of the Limelight lens to the floor
            */
			lensHeightInches = 20,
            
            /** 
            * Distance from the april tag to the floor
            */
            aprilTagToFloorInches = 17.12598;
	}

	public static class AprilTagCoords {
		/**
		 * direction: 1 (or !(2 or 3)) is fwd, 2 is left, 3 is right
		 */
		public static Translation2d TranslationFromAprilTagCoordinate(double in_id, int direction) {
			int id = (int) in_id;
			if (direction == 2) //left
				return new Translation2d(MAP[id][0], MAP[id][1] + 13.75);
			else if (direction == 3) //right
				return new Translation2d(MAP[id][0], MAP[id][1] - 13.75);
			else
				return new Translation2d(MAP[id][0], MAP[id][1]);
		}

		/**
		 * X, Y, Z, Heading (In that order)
		 */
		public static final double[][] MAP = {
				{
						582.49, 42.19, 18.22, 180
				},
				{
						582.49, 108.19, 18.22, 180
				},
				{
						582.49, 174.19, 18.22, 180
				},
				{
						636.96, 265.74, 27.38, 180
				},
				{
						14.25, 265.74, 27.38, 0
				},
				{
						68.73, 174.19, 18.22, 0
				},
				{
						68.73, 108.19, 18.22, 0
				},
				{
						68.73, 42.19, 18.22, 0
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
			 * Where the front left wheel is located in robot space
			 */
			public static final double
			FRONT_LEFT = 0.391,
			/**
			 * Where the front right wheel is located in robot space
			 */
			FRONT_RIGHT = 0.391,
			/**
			 * Where the back left  wheel is located in robot space
			 */
			BACK_LEFT = 0.391,
			/**
			 * Where the back right wheel is located in robot space
			 */
			BACK_RIGHT = 0.391;
		}
	}


	public static class Auton {
		/**
		 * The max forward speed of the robot in meters per second
		 */
		public static final double
		MAX_METRES_PER_SEC = .30, // 3.87096,
		/**
		 * The max acceleration of the robot in meters per second
		 */
		MAX_ACCEL = .30, // 2
		/**
		 * The prefered velocity in meters per second (The velocity the robot will use for auton)
		 */
		DRIVE_VEL = 8.5,
		/**
		 * The distance between the left and right wheels
		 */
		TRACK_WIDTH_METRES = 0,

		// TODO: Figure out this variable is used for
		VOLTS = 0.22,
		// TODO: Figure out this variable is used for
		VOLT_SECS_PER_M = 1.98,
		// TODO: Figure out this variable is used for
		VOLT_SECS_SQURED_PER_M = 1.98,
		// TODO: Figure out this variable is used for
		RAMSETE_B = 2,
		// TODO: Figure out this variable is used for
		RAMSETE_ZETA = 0.7,
		/**
		 * The distance (In feet) per each pulse on the encoder
		 */
		DISTANCE_PER_PULSE = 25.132741228718 / 8192;

		public static final java.util.HashMap<String, Command> EVENT_MAP = new java.util.HashMap<>();
	}


	public static class Arm {
		public static class Speeds {
			/**
			 * The arm speed multiplier
			 */
			public static final double
			MULTIPLIER = 1,
			// TODO: Figure out if this variable is used
			MULTIPLIER_IN = 1.0,
			// TODO: Figure out if this variable is used
			MULTIPLIER_OUT = 1.3;
		}

		public static class Ports {
			/**
			 * The CAN port the Pneumatics Control Module is on
			 */
			public static final int
			PCM = 0,
			/**
			 *  The CAN port the Top Arm Motor is on
			 */
			TOP_ARM = 3,
			/**
			 * The CAN port the Bottom Arm Motor is on
			 */
			BOTTOM_ARM = 4;
		}

		public static class Encoders {
			/**
			 * Channel A of the Arm Encoder
			 */
			public static final int
			CHANNEL_A = 0,
			/**
			 * Channel B of the Arm Encoder
			 */
			CHANNEL_B = 0;
		}

		public static class Limits {
			/**
			 * Encoder value that represents 180 degrees on the arm in physical space
			 */
			public static final int
			POS_180_DEG = 3500,
			/**
			 * Encoder value that represents the limit to which the arm can rotate backwards
			 */
			ARM_BACKWARDS_LIMIT = 6100,

			/**
			 * Encoder value that represents the limit to which the arm can rotate forwards
			 */
			ARM_FORWARD_LIMIT = 400;
		}

		public static class Claw {
			/**
			 * The port to the air valve that engages the solenoid.
			 */
			public static final int
			FWD_PORT = 0,
			/**
			 * The port to the air valve that disengages the solenoid.
			 */
			REVERSE_PORT = 0;
		}

		public static class Telescoper {
			/**
			 * The port to the air valve that disengages the solenoid.
			 */
			public static final int
			FWD_PORT = 0,
			/**
			 * The port to the air valve that disengages the solenoid.
			 */
			REVERSE_PORT = 0;
		}

		public static class Brake {
			/**
			 * The port to the air valve that disengages the solenoid.
			 */
			public static final int
			PORT_FWD = 0,
			/**
			 * The port to the air valve that disengages the solenoid.
			 */
			PORT_BACK = 0;
		}
	}


	public static class AutoBalance {
		/**
        * Rotation deadzone where the robot won't try to auto-balance
        */
		public static final double
		DEADZONE_MAX = 30,
		/**
        * Rotation deadzone where the robot won't try to auto-balance
        */
		DEADZONE_MIN = -30;

		public static class PID {
			/**
			 * The proportial gain of the auto balancing PID loop
			 */
			public static final double
			KP = .25, // Previous .7
			/**
			 * The integral gain of the auto balancing PID loop
			 */
			KI = 0,
			/**
			 * The derivative gain of the auto balancing PID loop
			 */
			KD = 0,
			/**
			 * The period between PID controller updates
			 */
			PERIOD = 0.02,

			/**
			* The tolerance the PID loops gives to be consided at the setpoint
			*/
			TARGET_TOLERANCE = 2,

			/**
            * The set value where the robot is level
            */
            SETPOINT = 0;
		}

		/**
		 * The range where if you push the controller joystick during AutoBalancing it will stop
		 */
		public static class DisableThresholds {
			/**
			 * The minimum/maximum values you can push the controller sticks to before canceling auto-balance
			 */
			public static final double
			LEFTSTICK_MAX = 0.1,
			LEFTSTICK_MIN = -0.1,
			RIGHTSTICK_MAX = 0.1,
			RIGHTSTICK_MIN = -0.1;
		}

		public static class Thresholds {
			/**
			 * Target Threshold Max
			 */
			public static final double
			TARGET_PITCH_MAX = 0,
			/**
			 * Target Threshold Min
			 */
			TARGET_PITCH_MIN = 0;
		}
	}

}