// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import javax.swing.TransferHandler;

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

  public static final int CAN_PORT_5 = 5;
  public static final int CAN_PORT_7 = 7;
  public static final int CAN_PORT_2 = 2;
  public static final int CAN_PORT_6 = 6;
  public static final int CAN_PORT_3 = 3;
  public static final int CAN_PORT_4 = 4;
  public static final int CAN_PORT_0 = 0;


  public static class ControllerConstants {
    public static final int DRIVERONE_PORT = 0;
    public static final int DRIVERTWO_PORT = 1;
  }

  public static class AprilTagCoordinates {
    /** direction: 1 (or !(2 or 3)) is fwd, 2 is left, 3 is right */
    public static final Translation2d AprilTagCoord_Trans2d(double in_id, int direction) {
      int id = (int)in_id;
      if (direction == 2) //left
      return new Translation2d(APRILTAG_COORDS[id][0], APRILTAG_COORDS[id][1] + 13.75);
      else if (direction == 3) //right
      return new Translation2d(APRILTAG_COORDS[id][0], APRILTAG_COORDS[id][1] - 13.75);

      return new Translation2d(APRILTAG_COORDS[id][0], APRILTAG_COORDS[id][1]);
    }
    

    public static final double[][] APRILTAG_COORDS = {
      {
        610.77 - 28.28, 42.19, 18.22, 180
      },
      {
        610.77 - 28.28, 108.19, 18.22, 180
      },
      {
        610.77 - 28.28, 174.19, 18.22, 180
      },
      {
        636.96, 265.74, 27.38, 180
      },
      {
        14.25, 265.74, 27.38, 0
      },
      {
        40.45 + 28.28, 174.19, 18.22, 0
      },
      {
        40.45 + 28.28, 108.19, 18.22, 0
      },
      {
        40.45 + 28.28, 42.19, 18.22, 0
      }
    };
  }

  public static class DriveTrainConstants {
    public static final int FRONT_LEFT_MOTOR_PORT = CAN_PORT_2;
    public static final int FRONT_RIGHT_MOTOR_PORT = CAN_PORT_7;
    public static final int BACK_LEFT_MOTOR_PORT = CAN_PORT_5;
    public static final int BACK_RIGHT_MOTOR_PORT = CAN_PORT_6;


    public static final int FRONT_LEFT_ENCODER_A = 4;
    public static final int FRONT_LEFT_ENCODER_B = 5;

    public static final int FRONT_RIGHT_ENCODER_A = 6;
    public static final int FRONT_RIGHT_ENCODER_B = 7;

    public static final int BACK_LEFT_ENCODER_A = 0;
    public static final int BACK_LEFT_ENCODER_B = 1;
    
    public static final int BACK_RIGHT_ENCODER_A = 2;
    public static final int BACK_RIGHT_ENCODER_B = 3;

    public static final double FRONT_LEFT_LOCATION = 0.391;
    public static final double FRONT_RIGHT_LOCATION = 0.391;
    public static final double BACK_LEFT_LOCATION = 0.391;
    public static final double BACK_RIGHT_LOCATION = 0.391;

   

   

    public static class DriveAuton {
      // #TODO# Add values to below constants
      // The forward maximum speed
      public static final double MAX_METRES_PER_SEC = .30; // 3.87096
      public static final double MAX_ACCEL = .30; // 2
      public static final double DRIVE_VEL = 8.5;

      public static final double TRACK_WIDTH_METRES = 0;

      public static final double VOLTS = 0.22;
      public static final double VOLT_SECS_PER_M = 1.98;
      public static final double VOLT_SECS_SQURED_PER_M = 1.98;

      public static final double RAMSETE_B = 2;
      public static final double RAMSETE_ZETA = 0.7;

      public static final  HashMap<String, Command> EVENT_MAP = new HashMap<>();

      public static final double DISTANCE_PER_PULSE = 25.132741228718 / 39.37;

    }
  }

  public static class ArmConstants {
    public static final int PCM_PORT = CAN_PORT_0;

    public static final int CLAW_FWD_PORT = 0;
    public static final int CLAW_RVRSE_PORT = 1;

    public static final int TELESCOPER_FWD_PORT = 2;
    public static final int TELESCOPER_RVRSE_PORT = 3;
    
    public static final int BRAKING_PORT_FWD = 5;
    public static final int BRAKING_PORT_BACK = 4;

    public static final int TOP_ARM_MOTOR_PORT = 3;
    public static final int BOTTOM_ARM_MOTOR_PORT = 4;

    public static final int ARM_ENCODER_CHANNEL_A = 9;
    public static final int ARM_ENCODER_CHANNEL_B = 8;
  }

  public static class VMXPiConstants {
    // Target Threshold Min and Max
    public static final double TARGET_PITCH_MAX_THRESHOLD = 0;
    public static final double TARGET_PITCH_MIN_THRESHOLD = 0;

    public static class AutoBalanceConstants {
      // Target Tolerance
      public static final double TARGET_TOLERANCE = 2;

      // Deadzone where the robot won't try to autobalance
      public static final double DEADZONE_MAX = 30;
      public static final double DEADZONE_MIN = -30;

      // The set value where the robot is level
      public static final double SETPOINT = 0;

      // The proportial gain of the auto balancing PID loop
      public static final double KP = .7;
      // The integral gain of the auto balancing PID loop
      public static final double KI = 0;
      // The derivitive gain of the auto balancing PID loop
      public static final double KD = 0;

      // The period between controller updates
      public static final double PID_PERIOD = 0.02;

      // The range where if you push the controller joystick during AutoBalancing it will stop
      public static final double DISABLETHRESHOLD_CONTROLLER_LEFTSTICK_MAX = 0.1;
      public static final double DISABLETHRESHOLD_CONTROLLER_LEFTSTICK_MIN = -0.1;

      public static final double DISABLETHRESHOLD_CONTROLLER_RIGHTSTICK_MAX = 0.1;
      public static final double DISABLETHRESHOLD_CONTROLLER_RIGHTSTICK_MIN = -0.1;
    }
  }
}
