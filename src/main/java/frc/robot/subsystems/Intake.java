// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Motors.Ports;

public class Intake extends SubsystemBase {
  public static class Motors {
    public static WPI_VictorSPX lift;
    public static CANSparkMax left, right;
  }

  public static class Encoders {
    public static Encoder lift;
  }

  public Intake() {
    Motors.left = new CANSparkMax(Ports.Intake.LEFT, MotorType.kBrushless);
    Motors.right = new CANSparkMax(Ports.Intake.RIGHT, MotorType.kBrushless);

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
    Motors.right.set(-speed);
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
   * Drive the Intake at speed to encoderPos
   * @param encoderPos The encoder position to drive to
   * @param speed The speed to drive the intake at
   */
  public static void driveLiftAuton(short encoderPos, float speed) {
      // TODO: Finish this
  }

  // /**
  //  * Drives the lift to whatever position is specified
  //  * @param position Which position to set the intake to (1 is pickup and 2 is shooter feeding)
  //  * @param speed The speed the lift motor drives at
  //  */
  // public static void driveLiftToPos(byte position, double speed) {

  //   speed = Math.abs(speed);

  //   switch (position) {
  //     // Pickup position
  //     case 1:
  //       // If the lift is below or at the pickup position stop, otherwise keep driving
  //       if (Encoders.lift.get() >= Constants.Encoders.Intake.PICKUP_POSITION_HIGHER)
  //         driveLift(0);
  //       else
  //         driveLift(-speed);

  //     break;

  //     // Amp scoring position
  //     // case 2:
  //     //   // If the lift is outside the amp scoring range keep driving
  //     //   if ((Encoders.lift.get() <= Constants.Encoders.Intake.AMP_SCORING_POSITION_UPPER) && (Encoders.lift.get() >= Constants.Encoders.Intake.AMP_SCORING_POSITION_LOWER))
  //     //     // If the lift is below the upper limit drive downwards if its not drive upwards
  //     //     if (Encoders.lift.get() <= Constants.Encoders.Intake.AMP_SCORING_POSITION_UPPER)
  //     //       driveLift(-1 * speed);
  //     //     else
  //     //       driveLift(speed);
  //     //   else
  //     //     driveLift(0);
      
  //     // break;

  //     // Shooter feeding position
  //     case 2:
  //       // If the lift is above or at the shooting position, stop it, otherwise keep driving upwards
  //       if (Encoders.lift.get() <= Constants.Encoders.Intake.SHOOTING_POSITION_LOWER)
  //         driveLift(0);
  //       else
  //         driveLift(speed);

  //     break;
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    SmartDashboard.putNumber("Intake Lift Encoder Pos", Encoders.lift.get());    
    SmartDashboard.putBoolean("Intake On", Intake.Motors.left.get() != 0);
  }
}
