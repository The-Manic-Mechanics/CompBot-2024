package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.HumanInterface.CustomController.ToySaxophone;

public class HumanInterface {
    // Reminder: Interface ports are zero-indexed.
    // Use the JoystickButtons to create routine triggers.
    // I wouldn't make any sort of routine triggers for controllers after the first.

    private static final XboxController port1_ = new XboxController(0);
    // private static final JoystickButton port1_bX = new JoystickButton(port1_, 3);
    private static final JoystickButton port1_bA = new JoystickButton(port1_, 1);
    // private static final JoystickButton port1_bB = new JoystickButton(port1_, 2);
    // private static final JoystickButton port1_bY = new JoystickButton(port1_, 4);
    // private static final XboxController port2_ = new XboxController(1);
    // private static final JoystickButton port2_bX = new JoystickButton(port2_, 3);
    // private static final JoystickButton port2_bA = new JoystickButton(port2_, 1);
    // private static final JoystickButton port2_bB = new JoystickButton(port2_, 2);
    // private static final JoystickButton port2_bY = new JoystickButton(port2_, 4);
    private static final GenericHID port3_ = new GenericHID(2);

    public class DriveMecanum {
        public static double getAxisX() {
            return port1_.getLeftX();
        }

        public static double getAxisY() {
            return port1_.getLeftY();
        }

        public static double getAxisZ() {
            return port1_.getRightX();
        }
    }

    public class CommandMap {
        public static void autoShooterAlign(Command execute) {
            // A button, interface on port 1.
            port1_bA.onTrue(execute);
        }
    }

    public class ShooterDrive {
        public static boolean activationDesired() {
            return port3_.getRawButton(ToySaxophone.RED);
        }
    }

    public class IntakeDrive {
        public static boolean forwardDesired() {
            return port3_.getRawButton(ToySaxophone.BLUE);
        }

        public static boolean backwardsDesired() {
            return port3_.getRawButton(ToySaxophone.ORANGE);
        }

        public static double getLiftDriveAxis() {
            return port3_.getRawAxis(ToySaxophone.AXIS_X);
        }
    }

    public class ClimberDrive {
        public static boolean hookForwardDesired() {
            return port3_.getRawButton(ToySaxophone.PINK);
        }

        public static boolean hookBackwardDesired() {
            return port3_.getRawButton(ToySaxophone.GREEN);
        }

        public static boolean climberDriveDesired() {
            return port3_.getRawButton(ToySaxophone.JOYSTICK);
        }

        public static boolean climberReverseDesired() {
            return port3_.getRawButton(ToySaxophone.GREEN);
        }
    }

    public static class CustomController {
        public static class ToySaxophone {
            public static final byte ORANGE = 1;
            public static final byte RED = 2;
            public static final byte BLUE = 3;
            public static final byte GREEN = 4;
            public static final byte SALMON = 5;
            public static final byte YELLOW = 6;
            public static final byte PINK = 7;
            public static final byte PURPLE = 8;
            public static final byte JOYSTICK = 13;
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
