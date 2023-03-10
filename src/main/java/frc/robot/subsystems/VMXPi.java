// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VMXPiConstants.AutoBalanceConstants;

public class VMXPi extends SubsystemBase {
  /** Creates a new VMXPi. */

  // Defining VMX Pi
  AHRS vmxPi;
  // Double to Store VMX Pi Roll
  double vmxPiRoll;
  // #TODO# Use the heading from the VMXPi to remain perpendicular to the switch when auto balancing
  double vmxPiHeading;

  // Defining autoBalancingPID
  PIDController autoBalancingPID;

  boolean autoBalanceCommandIsActive;
  
  double kP;
  double kI;
  double kD;

  public VMXPi() {

    // Initializing vmxPi  
    try { 
      vmxPi = new AHRS(SPI.Port.kMXP);
  } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
  }

  // #TODO#: Restore autoBalancing PID values back to constants
    // Initialising autoBalancingPID
    autoBalancingPID = new PIDController(kP, kI, kD, AutoBalanceConstants.PID_PERIOD);
    // Setting the setpoint for the autoBalancingPID
    autoBalancingPID.setSetpoint(AutoBalanceConstants.SETPOINT);
    // Setting the tolerence for the setpoint
    autoBalancingPID.setTolerance(AutoBalanceConstants.TARGET_TOLERANCE);
  }

  //Function to get pitch from VMXPI
  public double GetRollVMXPI() {
    // double vmxPitchy = vmxPi.getPitch();
    // return vmxPitchy;
    return vmxPi.getRoll();
  } 

  // Function for Calculating AutoBalance PID
  public double CalculateAutoBalancePID() {
    return  autoBalancingPID.calculate(vmxPi.getRoll());
  }

  // Function for checking if AutoBalancePID is at SetPoint
  public boolean AutoBalancePIDAtSetpoint() {
    return autoBalancingPID.atSetpoint();
  }

  public Rotation2d getRotation2d() {
    return vmxPi.getRotation2d();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Getting VMX Pi Roll and reporting it to the Shuffleboard under "VMX Pi Roll"
    vmxPiRoll = vmxPi.getRoll();

    SmartDashboard.putNumber("VMX Pi Roll (Less Fancy)", vmxPiRoll);

    // Putting PID value on the SmartDashboard
    SmartDashboard.putNumber("AutoBalancing PID Loop Output", autoBalancingPID.calculate(vmxPiRoll));
    // Putting all the data from the Autobalancing PID onto SmartDashboard
    SmartDashboard.putData("AutoBalancing PID", autoBalancingPID);

    SmartDashboard.getNumber("kP", kP);
    SmartDashboard.getNumber("kI", kI);
    SmartDashboard.getNumber("kD", kD);

    // #FIXME# Possibly an off number (Check to make sure it's right)
    SmartDashboard.putNumber("Accel X", vmxPi.getWorldLinearAccelX());
    SmartDashboard.putNumber("Accel Y", vmxPi.getWorldLinearAccelY());
    SmartDashboard.putNumber("Accel Z", vmxPi.getWorldLinearAccelZ());
  }
}
