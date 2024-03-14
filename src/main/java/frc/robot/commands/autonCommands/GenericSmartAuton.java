// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.ComplexAuton;

public class GenericSmartAuton extends SequentialCommandGroup {
  public GenericSmartAuton() {
    // FIXME: Order will change depending on path
    addCommands(
      // TODO: Finish this
      // Example
      // new InstantCommand(() -> {
      //   Intake.
      // }),
      ComplexAuton.createDriveCommand(Auton.trajectories[0])
    );
  }
}
