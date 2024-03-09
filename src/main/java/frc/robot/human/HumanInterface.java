package frc.robot.human;

public class HumanInterface {
    public static enum ControllerBrand {
        Generic,
        MicrosoftXbox,
        SonyDualshock,
        ManicMechanicsSaxophone
    } 

    public Object device;
    public ControllerBrand brand;

    public HumanInterface(Object device, ControllerBrand brand) {
        this.device = device;
        this.brand = brand;
    }
}