package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyroscope extends SubsystemBase {
    /**
     * The sensor used to obtain data.
     */
    public static AHRS sensor;
    /**
     * Stores the current pitch rotation of the robot obtained from the sensor.
     */
    public static double pitch;
    /**
     * Stores the current roll rotation of the robot obtained from the sensor.
     */ 
    public static double roll;
    /**
     * Stores the current yaw rotation of the robot obtained from the sensor.
     */
    public static double yaw;
    
	public Gyroscope() {
        sensor = new AHRS(Port.kMXP);
    }

    @Override
    public void periodic() {
        pitch = sensor.getPitch();
		roll = sensor.getRoll();
		yaw = sensor.getYaw();

		// SmartDashboard.putNumber("Pitch (Less Fancy)", pitch);
		// SmartDashboard.putNumber("Roll (Less Fancy)", roll);
		// SmartDashboard.putNumber("Yaw (Less Fancy)", yaw);
    }
}
