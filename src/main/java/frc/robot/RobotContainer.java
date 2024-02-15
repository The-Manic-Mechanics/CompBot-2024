// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controllers;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.IntakeDrive;
import frc.robot.commands.ShooterDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Gyroscope;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Solenoids;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> autoRoutineChooser;

  private final Solenoids sysSolenoids = new Solenoids();
  private final Gyroscope sysGyroscope = new Gyroscope();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final XboxController driverOneController = new XboxController(Controllers.DRIVERONE_PORT);
  public static final JoystickButton 
    mainButton1 = new JoystickButton(driverOneController, 1),
    mainButton2 = new JoystickButton(driverOneController, 4);

  public static final XboxController driverTwoController = new XboxController(Controllers.DRIVERTWO_PORT);
  public static final JoystickButton 
    driverSecondA = new JoystickButton(driverTwoController, 1),
    driverSecondY = new JoystickButton(driverTwoController, 4),
    driverSecondLeftBump = new JoystickButton(driverTwoController, 5),
    driverSecondRghtBump = new JoystickButton(driverTwoController, 6);

  public static final GenericHID saxController = new GenericHID(Constants.Controllers.SAX_PORT);
  // ---------------------------------------------------------------------------------

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // The robot's subsystems and commands are defined here...
    DriveTrain sysDriveTrain = new DriveTrain();
    Intake sysIntake = new Intake();
    Shooter sysShooter = new Shooter();

    ShooterDrive cmdShooterDrive = new ShooterDrive(sysShooter);
    IntakeDrive cmdIntakeDrive = new IntakeDrive(sysIntake);
    DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);

    sysShooter.setDefaultCommand(cmdShooterDrive);
    sysDriveTrain.setDefaultCommand(cmdDriveMecanum);
    sysIntake.setDefaultCommand(cmdIntakeDrive);
    // Configure the trigger bindings
    configureBindings();

  // -------------------------
  // SmartDashboard
  // -------------------------
  
    autoRoutineChooser = new SendableChooser<>();

    SmartDashboard.putData("Auton Chooser", autoRoutineChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Configure controller bindings here.
    // mainButton1.onTrue(cmdMove);
    // mainButton2.onTrue(cmdBrakeUp);

    // The DriveMecanum command periodically checks the controller's joysticks for acceleration.
  }

  public Command getAutonomousCommand() {
    return autoRoutineChooser.getSelected();
  }
}
