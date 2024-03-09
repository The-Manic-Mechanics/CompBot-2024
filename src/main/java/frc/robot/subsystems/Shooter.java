// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AutoShooterAlign;

public class Shooter extends SubsystemBase {
  public static class Motors {
    public static WPI_VictorSPX left, right;
  }

  public Shooter() {
    Motors.left = new WPI_VictorSPX(Constants.Motors.Ports.Shooter.LEFT);
    Motors.right = new WPI_VictorSPX(Constants.Motors.Ports.Shooter.RIGHT);

    Motors.right.setInverted(true);
  }

  public static void setSpeed(double speed) {
    Motors.left.set(speed);
    Motors.right.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter Alignment Active", AutoShooterAlign.isAligning);
  }
}
