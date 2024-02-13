// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    Encoders.lift = new Encoder(Constants.Encoders.Ports.Intake.LIFT_A, Constants.Encoders.Ports.Intake.LIFT_B);

    Encoders.lift.setDistancePerPulse(Constants.Intake.LIFT_DISTANCE_PER_PULSE);
  }

  public static void On(double speed) {
    Motors.left.set(speed);
    Motors.right.set(-1 * speed);
  }

  public static void Down() {
    // TODO: Fill this in
    if (Encoders.lift.get() >= 0)
      Motors.lift.set()
    else
      Motors.lift.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
