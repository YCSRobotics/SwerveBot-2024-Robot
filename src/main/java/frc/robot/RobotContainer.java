package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_operator = new CommandXboxController(1);

  double triggerThreshold = 0.7;

  // XboxController xboxTrigger = new Button(() -> xboxController.getLeftTriggerAxis() > triggerThreshold);
  // boolean xboxTrigger = Math.abs(operator.get()) > 0.1;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final ProximitySensorSubsystem proximitySensorSubsystem = new ProximitySensorSubsystem();

  private final HangerSubsystem leftHangerSubsystem = new HangerSubsystem(Constants.Mechanisms.leftHangerMotorID);
  private final HangerSubsystem rightHangerSubsystem = new HangerSubsystem(Constants.Mechanisms.rightHangerMotorID);

  /* Commands */
  private final ExampleAuto exampleAuto = new ExampleAuto(s_Swerve, launcherSubsystem, conveyorSubsystem);
  private final AutonomousLauncherCmd autonomousLauncherCmd = new AutonomousLauncherCmd(launcherSubsystem, 0.6, conveyorSubsystem, 1.0, 5.0);
  //private final LauncherCmd launcherCmd = new LauncherCmd(launcherSubsystem, Constants.Mechanisms.launcherTargetSpeed);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
   public AutonomousLauncherCmd getAutonomousLauncherCmd() {
    return new AutonomousLauncherCmd (launcherSubsystem, 0.6, conveyorSubsystem, 1.0, 5.0);
   } 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -m_driver.getLeftY(),
            () -> -m_driver.getLeftX(),
            () -> -m_driver.getRightX(),
            () -> m_driver.leftBumper().getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous commands
    configureAutonomousCommands();
  }

  private void configureAutonomousCommands() {
    autoChooser.setDefaultOption("Swerve + Launcher Auto", exampleAuto);
    autoChooser.addOption("Launcher Auto", autonomousLauncherCmd);
    SmartDashboard.putData("Autonomous Chooser", autoChooser);

    // Create a Shuffleboard layout for the auto buttons
    // ShuffleboardLayout autoLayout = Shuffleboard.getTab("Autonomous").getLayout("Auto Modes", BuiltInLayouts.kList);

    // Add buttons to the Shuffleboard layout for each autonomous option
    // autoLayout.add("Swerve + launcher Auto", exampleAuto);
    // autoLayout.add("Launcher Auto", autonomousLauncherCmd);

    // IMPORTANT: Update the following line to update the layout on Shuffleboard
    Shuffleboard.update();
  }

  public LauncherSubsystem getLauncherSubsystem() {
    return launcherSubsystem;
  }

  public ConveyorSubsystem getConveyorSubsystem() {
    return conveyorSubsystem;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    m_driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    
    // Modified 02/25/24 12:00pm
    m_driver.a().whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem));
    m_driver.leftBumper().whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem));
    m_driver.rightBumper().whileTrue(new ConveyorLauncherCmd(launcherSubsystem, conveyorSubsystem, proximitySensorSubsystem));

    m_operator.y().onTrue(new LauncherCmd(launcherSubsystem, 0.5));

    m_operator.a().whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem));

    //grabberConveyorButton.whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem, operator));
    m_operator.b().whileTrue(new ConveyorLauncherCmd(launcherSubsystem, conveyorSubsystem, proximitySensorSubsystem));
    // hangerButton.onTrue(new LiftCmd(hangerSubsystem, operator));

    // new Trigger(() -> xboxController.getLeftTriggerAxis() > triggerThreshold)
    //     .whenActive(new LiftCmd(leftHangerDown, Constants.Mechanisms.downVelocity, Constants.Mechanisms.leftLimitSwitch, 
    //                             null, () -> xboxController.getRightTriggerAxis() <= triggerThreshold));
    // }

    m_operator.leftTrigger(triggerThreshold)
        .whileTrue(new LowerCmd(leftHangerSubsystem, Constants.Mechanisms.downVelocity, Constants.Mechanisms.leftLimitSwitch));

    m_operator.rightTrigger(triggerThreshold)
        .whileTrue(new LowerCmd(rightHangerSubsystem, Constants.Mechanisms.downVelocity, Constants.Mechanisms.rightLimitSwitch));

    // Binding for the left hanger to lift up when the left bumper is pressed
    m_operator.leftBumper().whileTrue(new LiftCmd(leftHangerSubsystem, Constants.Mechanisms.upVelocity, 
                                               Constants.Mechanisms.leftLimitSwitch));

    // Binding for the right hanger to lift up when the right bumper is pressed
    m_operator.rightBumper().whileTrue(new LiftCmd(rightHangerSubsystem, Constants.Mechanisms.upVelocity, 
                                                Constants.Mechanisms.rightLimitSwitch));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new exampleAuto(s_Swerve);
    // return new AutonomousLauncherCmd(launcherSubsystem, 0.5, 5);
    // return exampleAuto;
    return autoChooser.getSelected();
  }
}
