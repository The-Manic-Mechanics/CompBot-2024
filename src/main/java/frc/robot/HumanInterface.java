package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.HumanInterface.CustomController.LogitechGamepad;

public class HumanInterface {
    // Reminder: Interface ports are zero-indexed.
    // Use the JoystickButtons to create routine triggers.
    // I wouldn't make any sort of routine triggers for controllers after the first.

    private static final XboxController port1_ = new XboxController(0);
    private static final JoystickButton port1_bX = new JoystickButton(port1_, 3);
    private static final JoystickButton port1_bA = new JoystickButton(port1_, 1);
    private static final JoystickButton port1_bB = new JoystickButton(port1_, 2);
    private static final JoystickButton port1_bY = new JoystickButton(port1_, 4);
    private static final GenericHID port2_ = new GenericHID(1);
    // private static final JoystickButton port2_bX = new JoystickButton(port2_, 3);
    // private static final JoystickButton port2_bA = new JoystickButton(port2_, 1);
    // private static final JoystickButton port2_bB = new JoystickButton(port2_, 2);
    // private static final JoystickButton port2_bY = new JoystickButton(port2_, 4);
    // private static final GeneericHID port3_ = new GenericHID(2);

    public class DriveMecanum {
        public static double getAxisX() {
            double a = -port1_.getRawAxis(LogitechGamepad.AXIS_LEFT_X);
            // double a =  -port1_.getRawAxis(XboxController.Axis.kLeftX.value);
            SmartDashboard.putNumber("Axis X", a);
            return a;
        }

        public static double getAxisY() {
            double a = port1_.getRawAxis(LogitechGamepad.AXIS_LEFT_Y);
            // double a = -port1_.getRawAxis(XboxController.Axis.kLeftY.value);
            SmartDashboard.putNumber("Axis Y", a);
            return a;
        }

        public static double getAxisZ() {
            double a = -port1_.getRawAxis(LogitechGamepad.AXIS_RIGHT_X);
            // double a = port1_.getRawAxis(XboxController.Axis.kRightX.value);
            SmartDashboard.putNumber("Axis Z", a);
            return a;
        }

        public static void smartDashboardDebugPut() {
            // SmartDashboard.putNumber("Axis Left X", port1_.getRawAxis(LogitechGamepad.AXIS_LEFT_X));
            // SmartDashboard.putNumber("Axis Left Y", port1_.getRawAxis(LogitechGamepad.AXIS_LEFT_Y));
            // SmartDashboard.putNumber("Axis Right X", port1_.getRawAxis(LogitechGamepad.AXIS_RIGHT_X));
            // SmartDashboard.putNumber("Axis Right Y", port1_.getRawAxis(LogitechGamepad.AXIS_RIGHT_Y));
            // SmartDashboard.putNumber("Axis Trigger Left", port1_.getRawAxis(LogitechGamepad.AXIS_LEFT_TRIGGER));
            // SmartDashboard.putNumber("Axis Trigger Right", port1_.getRawAxis(LogitechGamepad.AXIS_RIGHT_TRIGGER));
            SmartDashboard.putNumber("Axis Left X", port1_.getRawAxis(XboxController.Axis.kLeftX.value));
            SmartDashboard.putNumber("Axis Left Y", port1_.getRawAxis(XboxController.Axis.kLeftY.value));
            SmartDashboard.putNumber("Axis Right X", port1_.getRawAxis(XboxController.Axis.kRightX.value));
            SmartDashboard.putNumber("Axis Right Y", port1_.getRawAxis(XboxController.Axis.kRightY.value));
            SmartDashboard.putNumber("Axis Trigger Left", port1_.getRawAxis(XboxController.Axis.kLeftTrigger.value));
            SmartDashboard.putNumber("Axis Trigger Right", port1_.getRawAxis(XboxController.Axis.kRightTrigger.value));
            SmartDashboard.updateValues();
        }
    }

    public class CommandMap {
        public static void runSysIDQuasistaticForwards(Command execute) {
            port1_bY.onTrue(execute);
        }

        public static void runSysIDQuasistaticBackwards(Command execute) {
            port1_bA.onTrue(execute);
        }

        public static void runSysIDDynamicForwards(Command execute) {
            port1_bB.onTrue(execute);
        }

        public static void runSysIDDynamicBackwards(Command execute) {
            port1_bX.onTrue(execute);
        }
    }

    public class ShooterDrive {
        public static boolean activationDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.RED);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.RIGHT_BUMPER);
        }
    }

    public class IntakeDrive {
        public static boolean outDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.BLUE);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.LEFT_BUMPER);
        }

        public static boolean inDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.ORANGE);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.Y);
        }

        public static double getLiftDriveAxis() {
            // + Saxophone
            // return port3_.getRawAxis(ToySaxophone.AXIS_X);
            // + Generic
            return port2_.getRawAxis(LogitechGamepad.AXIS_LEFT_Y);
        }

        public static boolean ignoreLiftLimitsDesired() {
            boolean a = port2_.getRawAxis(LogitechGamepad.AXIS_LEFT_TRIGGER) > .8;
            SmartDashboard.putBoolean("Lift Limits Ignored", a);
            return a;
        }
    }

    public class ClimberDrive {
        public static boolean hookForwardDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.PINK);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.X);
        }

        public static boolean hookBackwardDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.PURPLE);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.A);
        }

        public static boolean climberDriveDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.JOYSTICK);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.START);
        }

        public static boolean climberReverseDesired() {
            // + Saxophone
            // return port3_.getRawButton(ToySaxophone.GREEN);
            // + Generic
            return port2_.getRawButton(LogitechGamepad.BACK);
        }
    }

    public static class CustomController {
        // Y-axises are inverted
        public static class LogitechGamepad {
            public static final byte A = 1;
            public static final byte B = 2;
            public static final byte X = 3;
            public static final byte Y = 4;
            public static final byte LEFT_BUMPER = 5;
            public static final byte RIGHT_BUMPER = 6;
            public static final byte BACK = 7;
            public static final byte START = 8;
            public static final byte LEFT_JOYSTICK = 9;
            public static final byte RIGHT_JOYSTICK = 10;
            public static final byte AXIS_LEFT_X = 0;
            public static final byte AXIS_LEFT_Y = 1;
            public static final byte AXIS_LEFT_TRIGGER = 2;
            public static final byte AXIS_RIGHT_TRIGGER = 3;
            public static final byte AXIS_RIGHT_X = 4;
            public static final byte AXIS_RIGHT_Y = 5;
        }

        public static class ToySaxophone {
            public static final byte ORANGE = 1;
            public static final byte RED = 2;
            public static final byte BLUE = 3;
            public static final byte GREEN = 4;
            public static final byte SALMON = 5;
            public static final byte YELLOW = 6;
            public static final byte PINK = 7;
            public static final byte PURPLE = 8;
            public static final byte JOYSTICK = 12;
            /**
             * The X axis on the joystick, towards the bell for positive values and away
             * from it for negative values.
             */
            public static final byte AXIS_X = 0;
            /**
             * The Y axis on the joystick, towards the buttons for positive values and away
             * from them for negative values.
             */
            public static final byte AXIS_Y = 1;
        }
    }
}
