// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArmDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.ClawClose;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.DriveMecanum;
import frc.robot.commands.DumbAuton;
import frc.robot.commands.TelescoperIn;
import frc.robot.commands.TelescoperOut;
import frc.robot.commands.BrakeDown;
import frc.robot.commands.BrakeUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Solenoids;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  // Subsystem
  // ---------------------------

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveTrain sysDriveTrain = new DriveTrain();

  private final Arm sysArm = new Arm();

  private final Solenoids sysSolenoids = new Solenoids();

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
    //     sysDriveTrain.mecanumDriveOdometry::getPoseMeters, // Pose2d supplier
    //     sysDriveTrain.mecanumDriveOdometry::resetPose, // Pose2d consumer,used to reset odometry at the beginning of auto
    //     sysDriveTrain.mecanumDriveKinematics, // SwerveDriveKinematics
    //     new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    //     new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    //     Autonomous.MAX_METRES_PER_SEC, // Module states consumer used to output to the drive subsystem
    //     sysDriveTrain :: setWheelSpeeds,
    //     eventMap,
    //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //     Subsystem.sysDriveTrain // The drive subsystem. Used to properly set the requirements of path following commands
    // );

  private final ArmDrive cmdArmDrive = new ArmDrive(sysArm/* , sysSolenoids*/);

  private final ClawOpen cmdClawOpen = new ClawOpen(sysSolenoids);
  private final ClawClose cmdClawClose = new ClawClose(sysSolenoids);

  private final TelescoperIn cmdTelescoperIn = new TelescoperIn(sysSolenoids, sysArm);
  private final TelescoperOut cmdTelescoperOut = new TelescoperOut(sysSolenoids, sysArm);

  private final DumbAuton cmdDumbAuton = new DumbAuton(sysDriveTrain, sysArm);

  private final BrakeUp cmdBrakeUp = new BrakeUp(sysSolenoids);
  private final BrakeDown cmdBrakeDown = new BrakeDown(sysSolenoids);


  // ----------------------------------------------------------------------------------

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
    return cmdDumbAuton;
  }
}
