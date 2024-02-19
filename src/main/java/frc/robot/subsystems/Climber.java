// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  public static class Motors {
    public static CANSparkMax one, two;

    /**
     * The motor that flips the hook into position
     */
    public static WPI_VictorSPX hookPositioner;
  }

  /** Creates a new Climber. */
  public Climber() {
    Motors.one = new CANSparkMax(Constants.Motors.Ports.Climber.ONE, MotorType.kBrushless);
    Motors.two = new CANSparkMax(Constants.Motors.Ports.Climber.TWO, MotorType.kBrushless);
    
    Motors.hookPositioner = new WPI_VictorSPX(Constants.Motors.Ports.Climber.HOOK_POSITIONER);

    Motors.two.follow(Motors.one, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
