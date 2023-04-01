// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.AutoBalanceAuton;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.PlaceDriveFwd;
import frc.robot.commands.TelescoperIn;
import frc.robot.commands.TelescoperOut;
import frc.robot.commands.BrakeDown;
import frc.robot.commands.BrakeUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Solenoids;
import frc.robot.subsystems.VMXPi;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
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
  // ---------------------------
  // Misc Vars
  // ---------------------------
  SendableChooser<Command> autoRoutineChooser;

  // ----------------------------------------------------------------------------------

  // ---------------------------
  // Subsystem
  // ---------------------------

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrain sysDriveTrain = new DriveTrain();

  private final Arm sysArm = new Arm();

  private final Solenoids sysSolenoids = new Solenoids();
  private final VMXPi sysVMXPi = new VMXPi();

  // ----------------------------------------------------------------------------------

  // ---------------------------
  // Commands
  // ---------------------------

  private final DriveMecanum cmdDriveMecanum = new DriveMecanum(sysDriveTrain);

  // This is just an example event map. It would be better to have a constant, global event map
  //   in your code that will be used by all path following commands.
  //   HashMap<String, Command> eventMap = new HashMap<>();

  //   // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    // MecanumAutoBuilder autoBuilder = new MecanumAutoBuilder(
    //     sysDriveTrain.mecanumDriveOdometry::getP SendableChooser<Command> autoRoutineChooser;
    //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //     Subsystem.sysDriveTrain // The drive subsystem. Used to properly set the requirements of path following commands
    // );

  private final ArmDrive cmdArmDrive = new ArmDrive(sysArm/* , sysSolenoids*/);

  private final ClawOpen cmdClawOpen = new ClawOpen(sysSolenoids);
  private final ClawClose cmdClawClose = new ClawClose(sysSolenoids);

  private final TelescoperIn cmdTelescoperIn = new TelescoperIn(sysSolenoids, sysArm);
  private final TelescoperOut cmdTelescoperOut = new TelescoperOut(sysSolenoids, sysArm);

  private final PlaceDriveFwd cmdPlaceDriveFwd = new PlaceDriveFwd(sysDriveTrain, sysArm, sysSolenoids);

  private final AutoBalanceAuton cmdAutoBalanceAuton = new AutoBalanceAuton(sysDriveTrain, sysVMXPi, sysSolenoids, sysArm);

  private final BrakeUp cmdBrakeUp = new BrakeUp(sysSolenoids);
  private final BrakeDown cmdBrakeDown = new BrakeDown(sysSolenoids);

  // ---------------------------
  // Controller
  // ---------------------------

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final XboxController driverMainController = new XboxController(ControllerConstants.DRIVERONE_PORT);
  private final JoystickButton mainButton1 = new JoystickButton(driverMainController, 1);
  private final JoystickButton mainButton2 = new JoystickButton(driverMainController, 4);

  public static final XboxController driverSecondController = new XboxController(ControllerConstants.DRIVERTWO_PORT);
  private final JoystickButton driverSecondA = new JoystickButton(driverSecondController, 1);
  private final JoystickButton driverSecondY = new JoystickButton(driverSecondController, 4);
  private final JoystickButton driverSecondLeftBump = new JoystickButton(driverSecondController, 5);
  private final JoystickButton driverSecondRghtBump = new JoystickButton(driverSecondController, 6);
  // ---------------------------------------------------------------------------------

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

  // --------------------------
  // SetDefaultCommand
  // --------------------------

  sysDriveTrain.setDefaultCommand(cmdDriveMecanum);
  sysArm.setDefaultCommand(cmdArmDrive);

  // ----------------------------------------------------------------------------------

  
    // Configure the trigger bindings
    configureBindings();

  // -------------------------
  // SmartDashboard
  // -------------------------
  
  autoRoutineChooser = new SendableChooser<>();
  
  autoRoutineChooser.addOption("AutoBalanceAuton", cmdAutoBalanceAuton);
  autoRoutineChooser.addOption("PlaceDriveForward", cmdPlaceDriveFwd);

  SmartDashboard.putData("Auton Chooser", autoRoutineChooser);

  // ----------------------------------------------------------------------------------
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

  // ----------------------------
  // Driver (Main)
  // ----------------------------
    // Assigning driver main button X to cmdAutoBalance
    mainButton1.onTrue(cmdBrakeDown);
    mainButton2.onTrue(cmdBrakeUp);



  // -------------------------------------------------------------------------------------

  
  // ----------------------------
  // Driver (Secondary)
  // ----------------------------

    driverSecondA.onTrue(cmdTelescoperIn);
   
    driverSecondY.onTrue(cmdTelescoperOut);

    

    driverSecondLeftBump.onTrue(cmdClawOpen);
    driverSecondRghtBump.onTrue(cmdClawClose);

    


  
  // -------------------------------------------------------------------------------------
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Rob
    
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    return autoRoutineChooser.getSelected();
  }
}
