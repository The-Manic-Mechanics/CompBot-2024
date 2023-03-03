// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.security.auth.login.FailedLoginException;

import edu.wpi.first.wpilibj.CAN;

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

  public static class DriveTrainConstants {
    public static final int FRONT_LEFT_MOTOR_PORT = CAN_PORT_2;
    public static final int FRONT_RIGHT_MOTOR_PORT = CAN_PORT_7;
    public static final int BACK_LEFT_MOTOR_PORT = CAN_PORT_5;
    public static final int BACK_RIGHT_MOTOR_PORT = CAN_PORT_6;
  }

  public static class ArmConstants {
    public static final int PCM_PORT = CAN_PORT_0;

    public static final int CLAW_FWD_PORT = 0;
    public static final int CLAW_RVRSE_PORT = 1;

    public static final int TELESCOPER_FWD_PORT = 2;
    public static final int TELESCOPER_RVRSE_PORT = 3;
    
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
