package frc.robot.human;

public class ManicMechanicsSaxophone extends HumanInterface {
    public ManicMechanicsSaxophone(Object device) {
        super(device, ControllerBrand.ManicMechanicsSaxophone);
    }

    public static class Button {
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

    public static class Axis {
        public static final byte X = 0;
        public static final byte Y = 1;
    }
}
