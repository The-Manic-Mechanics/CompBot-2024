// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  public static class Motors {
    public static WPI_VictorSPX left, right, lift;
  }

  public static class Encoders {
    public static Encoder lift;
  }

  /** Creates a new Intake. */
  public Intake() {
    Motors.left = new WPI_VictorSPX(Constants.Motors.Ports.Intake.LEFT);
    Motors.right = new WPI_VictorSPX(Constants.Motors.Ports.Intake.RIGHT);
    Motors.lift = new WPI_VictorSPX(Constants.Motors.Ports.Intake.LIFT);

    Motors.right.setInverted(true);

    Encoders.lift = new Encoder(Constants.Encoders.Ports.Intake.LIFT_A, Constants.Encoders.Ports.Intake.LIFT_B);

    Encoders.lift.setDistancePerPulse(frc.robot.Constants.Encoders.Intake.LIFT_DISTANCE_PER_PULSE);
  }

  /**
   * Turns on the intake at "speed"
   */
  public static void On(double speed) {
    Motors.left.set(speed);
    Motors.right.set(speed);
  }

  /**
   * Drives the lift at "speed"
   */
  public static void DriveLift(double speed) {
    // TODO: Fill this in
    if ((Encoders.lift.get() >= Constants.Encoders.Intake.INTAKE_LOWER_LIMIT) && (RobotContainer.driverTwoController.getLeftY() < 0)) 
      Motors.lift.set(0);
    else if ((Encoders.lift.get() <= Constants.Encoders.Intake.INTAKE_HIGH_LIMIT) && (RobotContainer.driverTwoController.getLeftY() > 0)) 
      Motors.lift.set(0);
    else 
      Motors.lift.set(speed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DriveLift(RobotContainer.driverTwoController.getLeftY() * Constants.Intake.LIFT_SPEED_MULTIPLIER);

    if (RobotContainer.driverTwoController.getAButton())
      On(Constants.Intake.SPEED);
    
    if (RobotContainer.driverTwoController.getYButton())
      On(-1 * Constants.Intake.SPEED);

    SmartDashboard.putNumber("Intake Lift Encoder Pos", Encoders.lift.get());
  }
}
