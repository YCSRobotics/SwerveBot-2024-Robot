package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  XboxController xboxController = new XboxController(2);

  int axisNumber = 2;
  JoystickButton operatorButton = new JoystickButton(operator, axisNumber);
  final int leftTriggerAxis = 2;
  double triggerThreshold = 0.7;


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Operator Buttons */
  private final JoystickButton launchButton =
      new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton grabberConveyorButton =
      new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton conveyorLauncherButton =
      new JoystickButton(operator, XboxController.Button.kB.value);
  private final JoystickButton hangerButton =
      new JoystickButton(operator, XboxController.Button.kX.value);

  private final JoystickButton leftHangerUpButton =
      new JoystickButton(operator, XboxController.Button.kLeftBumper);
  private final JoystickButton rightHangerUpButton =
      new JoystickButton(operator, XboxController.Button.kRightBumper);

  // XboxController xboxTrigger = new Button(() -> xboxController.getLeftTriggerAxis() > triggerThreshold);
  // boolean xboxTrigger = Math.abs(operator.get()) > 0.1;

  double leftHangerDown = xboxController.getLeftTriggerAxis();
  double rightHangerDown = xboxController.getRightTriggerAxis();

  leftHanger = new HangerSubsystem(Constants.Mechanisms.leftHangerMotorID);



  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final ProximitySensorSubsystem proximitySensorSubsystem = new ProximitySensorSubsystem();
  private final HangerSubsystem hangerSubsystem = new HangerSubsystem();

  /* Commands */
  private final ExampleAuto exampleAuto = new ExampleAuto(s_Swerve, launcherSubsystem, conveyorSubsystem);
  private final AutonomousLauncherCmd autonomousLauncherCmd = new AutonomousLauncherCmd(launcherSubsystem, 0.6, conveyorSubsystem, 1.0, 5.0);
  private final LauncherCmd launcherCmd = new LauncherCmd(launcherSubsystem, Constants.Mechanisms.launcherTargetSpeed);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
   public AutonomousLauncherCmd getAutonomousLauncherCmd() {
    return new AutonomousLauncherCmd (launcherSubsystem, 0.6, conveyorSubsystem, 1.0, 5.0);
   } 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

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
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    launchButton.onTrue(new LauncherCmd(launcherSubsystem, 0.5));


    grabberConveyorButton.whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem, operator, proximitySensorSubsystem));
    //grabberConveyorButton.whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem, operator));
    conveyorLauncherButton.whileTrue(new ConveyorLauncherCmd(launcherSubsystem, conveyorSubsystem, operator, proximitySensorSubsystem));
    hangerButton.onTrue(new LiftCmd(hangerSubsystem, operator));

    new Trigger(() -> xboxController.getLeftTriggerAxis() > triggerThreshold)
        .whenActive(new LiftCmd(leftHangerDown, Constants.Mechanisms.downVelocity, Constants.Mechanisms.leftLimitSwitch, 
                                null, () -> xboxController.getRightTriggerAxis() <= triggerThreshold));
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
