// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoBalance;

/**
* Holds everything pertaining to the NavX
*/
public class NavX extends SubsystemBase {
	/**
	 * The NavX sensor.
	 *
	*/
	public static AHRS sensor;
	/**
	* Stores pitch
	*/
	public static double
	    pitch,
	    /**
	    * Stores roll
	    */
	    roll,
	    /**
	    * Stores yaw
	    */
	    yaw;

	/**
	* The PID used to control autoBalancing
	*/
	private static PIDController autoBalancingPID;

	public static boolean autoBalanceCommandIsActive;

    /**
    * The proportional gain of the PID loop
    */
	private static double
	    kP,
	    /**
	    * The Integral gain of the PID loop
	    */
	    kI,
	    /**
        * The Integral gain of the PID loop
        */
	    kD;

	public NavX() {

		// Initializing NavX
		try {
			sensor = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		// TODO: Restore autoBalancing PID values back to constants
		// Initialising autoBalancingPID
		autoBalancingPID = new PIDController(kP, kI, kD, AutoBalance.PID.SETPOINT);
		// Setting the setpoint for the autoBalancingPID
		autoBalancingPID.setSetpoint(AutoBalance.PID.SETPOINT);
		// Setting the tolerance for the public boolean balanceRunning
		autoBalancingPID.setTolerance(AutoBalance.PID.TARGET_TOLERANCE);
	}

	/**
	* Function for Calculating AutoBalance PID
	*/
	public static double CalculateAutoBalancePID() {
		return autoBalancingPID.calculate(sensor.getPitch());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// Getting Roll and reporting it to the Shuffleboard
		pitch = sensor.getPitch();
		roll = sensor.getRoll();
		yaw = sensor.getYaw();
		SmartDashboard.putNumber("Pitch (Less Fancy)", pitch);
		SmartDashboard.putNumber("Roll (Less Fancy)", roll);
		SmartDashboard.putNumber("Yaw (Less Fancy)", yaw);

		// Putting PID value on the SmartDashboard
		SmartDashboard.putNumber("Auto-Balancing PID Loop Output", autoBalancingPID.calculate(pitch));
		// Putting all the data from the Auto-balancing PID onto SmartDashboard
		SmartDashboard.putData("Auto-Balancing PID", autoBalancingPID);

		kP = SmartDashboard.getNumber("kP", 0);
		kI = SmartDashboard.getNumber("kI", 0);
		kD = SmartDashboard.getNumber("kD", 0);

		// FIXME: Possibly an off number (Check to make sure it's right)
		SmartDashboard.putNumber("Accel X", sensor.getWorldLinearAccelX());
		SmartDashboard.putNumber("Accel Y", sensor.getWorldLinearAccelY());
		SmartDashboard.putNumber("Accel Z", sensor.getWorldLinearAccelZ());

		SmartDashboard.putBoolean("Auto-balance Active", autoBalanceCommandIsActive);
	}
}
