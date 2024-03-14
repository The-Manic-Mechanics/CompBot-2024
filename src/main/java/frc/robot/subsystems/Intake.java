// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HumanInterface;

public class Intake extends SubsystemBase {
  public static class Motors {
    public static WPI_VictorSPX left, right, lift;
  }

  public static class Encoders {
    public static Encoder lift;
  }

  public Intake() {
    Motors.left = new WPI_VictorSPX(Constants.Motors.Ports.Intake.LEFT);
    Motors.right = new WPI_VictorSPX(Constants.Motors.Ports.Intake.RIGHT);
    Motors.lift = new WPI_VictorSPX(Constants.Motors.Ports.Intake.LIFT);
    Motors.right.setInverted(true);
    Encoders.lift = new Encoder(Constants.Encoders.Ports.Intake.LIFT_A, Constants.Encoders.Ports.Intake.LIFT_B);
    Encoders.lift.setDistancePerPulse(Constants.Encoders.Intake.LIFT_DISTANCE_PER_PULSE);
    Encoders.lift.reset();
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
    // If the lift is below the lower limit or above the higher limit stop the lift from being driven and made it so it can only driven the way oppoisite the limit 
    // if (
    // (Encoders.lift.get() >= Constants.Intake.LOW_LIMIT) && (RobotContainer.saxController.getRawAxis(AxisPort.X) < 0) 
    // || 
    // (Encoders.lift.get() <= Constants.Intake.HIGH_LIMIT) && (RobotContainer.saxController.getRawAxis(AxisPort.X) > 0)
    // ) 
    //   Motors.lift.set(0);
    // else
      Motors.lift.set(speed);
  }

  /**
   * Drives the lift to whatever position is specified
   * @param position Which position to set the intake to (1 is pickup and 2 is shooter feeding)
   * @param speed The speed the lift motor drives at
   */
  public static void driveLiftToPos(int position, double speed) {

    speed = Math.abs(speed);

    switch (position) {
      // Pickup position
      case 1:
        // If the lift is below or at the pickup position stop, otherwise keep driving
        if (Encoders.lift.get() >= Constants.Encoders.Intake.PICKUP_POSITION_HIGHER)
          driveLift(0);
        else
          driveLift(-speed);

      break;

      // Amp scoring position
      // case 2:
      //   // If the lift is outside the amp scoring range keep driving
      //   if ((Encoders.lift.get() <= Constants.Encoders.Intake.AMP_SCORING_POSITION_UPPER) && (Encoders.lift.get() >= Constants.Encoders.Intake.AMP_SCORING_POSITION_LOWER))
      //     // If the lift is below the upper limit drive downwards if its not drive upwards
      //     if (Encoders.lift.get() <= Constants.Encoders.Intake.AMP_SCORING_POSITION_UPPER)
      //       driveLift(-1 * speed);
      //     else
      //       driveLift(speed);
      //   else
      //     driveLift(0);
      
      // break;

      // Shooter feeding position
      case 2:
        // If the lift is above or at the shooting position, stop it, otherwise keep driving upwards
        if (Encoders.lift.get() <= Constants.Encoders.Intake.SHOOTING_POSITION_LOWER)
          driveLift(0);
        else
          driveLift(speed);

      break;
    }
  }

  /**
   * Turns on the intake if the lift encoder goes past a certain threshold
   */
  public static void driveIntakeAuto() {

    if (Encoders.lift.get() >= Constants.Encoders.Intake.ON_LIMIT)
      setSpeed(frc.robot.Constants.Intake.SPEED);
    // If the intake is not running in reverse and the intake drive button is not being pressed, stop the motor.
    else if ((Motors.left.get() > 0) && !HumanInterface.IntakeDrive.outDesired())
      setSpeed(0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    SmartDashboard.putNumber("Intake Lift Encoder Pos", Encoders.lift.get());    
  }
}
