// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  public static class Motors {
    public static WPI_VictorSPX left, right;
  }

  /** Creates a new Shooter. */
  public Shooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    Motors.left = new WPI_VictorSPX(Constants.Motors.Ports.Shooter.LEFT);
    Motors.right = new WPI_VictorSPX(Constants.Motors.Ports.Shooter.RIGHT);

    Motors.right.setInverted(true);
  }

  public static void On(double speed) {
    Motors.left.set(speed);
    Motors.right.set(speed);
  }

  @Override
  public void periodic() {
    if (RobotContainer.driverTwoController.getAButtonPressed()) {
      On(Constants.Shooter.SPEED);
    }
  }
}
