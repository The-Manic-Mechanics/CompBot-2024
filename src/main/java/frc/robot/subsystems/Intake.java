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
import frc.robot.Constants.Controllers.Sax.AxisPort;

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
  public static void setSpeed(double speed) {
    Motors.left.set(speed);
    Motors.right.set(speed);
  }

  /**
   * Drives the lift at "speed" 
   * @param speed The speed to drive the lift at
   */
  public static void driveLift(double speed) {
    // TODO: Fill this in
    if ((Encoders.lift.get() >= Constants.Encoders.Intake.LOWER_LIMIT) && (-RobotContainer.saxController.getRawAxis(AxisPort.X) < 0)) 
      Motors.lift.set(0);
    else if ((Encoders.lift.get() <= Constants.Encoders.Intake.HIGH_LIMIT) && (RobotContainer.driverTwoController.getLeftY() > 0)) 
      Motors.lift.set(0);
    else 
      Motors.lift.set(speed);
  }

  /**
   * Drives the lift to whatever position is specified
   * @param position Which position to set the intake to (1 is pickup, 2 is amp scoring, and 3 is shooter feeding)
   * @param speed The speed the lift motor drives at
   */
  public static void driveLiftToPos(int position, double speed) {
    switch (position) {
      // Pickup position
      case 1:
      if (Encoders.lift.get() <= Constants.Encoders.Intake.ON_LIMIT)
        driveLift(0);
      else
        driveLift(speed);

      break;

      // Amp scoring position
      case 2:
      if ((Encoders.lift.get() <= Constants.Encoders.Intake.AMP_SCORING_POSITION_UPPER) && (Encoders.lift.get() >= Constants.Encoders.Intake.AMP_SCORING_POSITION_LOWER))
        driveLift(speed);
      else
        driveLift(0);
      
      break;

      // Shooter feeding position
      case 3:
      if (Encoders.lift.get() <= Constants.Encoders.Intake.SHOOTER_ON_LIMIT)
        driveLift(0);
      else
        driveLift(speed);

      break;

      // No position fed
      default:
    }
  }

  /**
   * Turns on the intake if the lift encoder goes past a certain threshold
   */
  public static void driveIntakeAuto() {
    // FIXME: Signs may be backwards
    if (Encoders.lift.get() <= Constants.Encoders.Intake.ON_LIMIT)
      setSpeed(Constants.Intake.SPEED);
    else if (Motors.left.get() > 0)
      setSpeed(0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Lift Encoder Pos", Encoders.lift.get());    
  }
}
