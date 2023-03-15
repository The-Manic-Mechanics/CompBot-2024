// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VMXPiConstants.AutoBalanceConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VMXPi;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final VMXPi sysVmxPi;
  private final DriveTrain sysDriveTrain;

  public AutoBalance(VMXPi inSysVMXPi, DriveTrain inSysDriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    sysVmxPi = inSysVMXPi;
    sysDriveTrain = inSysDriveTrain;
    addRequirements(sysVmxPi, sysDriveTrain);
  }
  
  public boolean isBalanced;

  // Current roll of the VMX Pi 
  public double currentRoll;

  // Speed for the motors
  double speed;
  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBalanced = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Getting the roll from the VMX Pi
     currentRoll = sysVmxPi.GetRollVMXPI();

     speed = sysVmxPi.CalculateAutoBalancePID();

    // #TODO# Could be a problem (Stopping once VMX Pi reads zero)
    // Checks if AutoBalancing is complete
    if (sysVmxPi.AutoBalancePIDAtSetpoint()) {
      isBalanced = true;
    } else {
      isBalanced = false;
    }

    if (!isBalanced) {
      sysDriveTrain.CartisianDrive(0, speed, 0);
    } 
    
    // while (!isBalanced) {

    //   speed = sysDriveTrain.CalculateAutoBalancePID();

    //   sysDriveTrain.DriveSpeeds(speed, speed);
    // }
    
  }
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sysDriveTrain.CartisianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((AutoBalanceConstants.DEADZONE_MIN >= currentRoll) 
    ||
    (AutoBalanceConstants.DEADZONE_MAX <= currentRoll)) {
      return true;
    } else if (isBalanced) {
      return true;

    // #TODO# Move this shutoff into RobotContainer or make it faster in some other way
    } else if (RobotContainer.driverMainController.getBButtonPressed() == true) {
      return true;

    } else if (
      (
      RobotContainer.driverMainController.getLeftY() > AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_LEFTSTICK_MAX 
      ||
      RobotContainer.driverMainController.getLeftY() < AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_LEFTSTICK_MIN
      ) || (
      RobotContainer.driverMainController.getRightX() > AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_RIGHTSTICK_MAX
      ||
      RobotContainer.driverMainController.getRightX() < AutoBalanceConstants.DISABLETHRESHOLD_CONTROLLER_RIGHTSTICK_MIN
      )
      ) {
      return true;

    } else {
      return false;
    }
  }

 



}
