package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final TrapArmSubsystem trapArmSubsystem = new TrapArmSubsystem();

  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();

  /* Commands */
  private final Side_RedNoBluePole side_RedNoBluePole = new Side_RedNoBluePole(s_Swerve, launcherSubsystem, grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  private final Side_RedPoleBlueNo side_RedPoleBlueNo = new Side_RedPoleBlueNo(s_Swerve, launcherSubsystem, grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  private final Center_Middle center_Middle = new Center_Middle(s_Swerve, launcherSubsystem, grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  private final Center_Middle_RedNoBluePole center_Middle_RedNoBluePole = new Center_Middle_RedNoBluePole(s_Swerve, launcherSubsystem, grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  private final Center_Middle_RedPoleBlueNo center_Middle_RedPoleBlueNo = new Center_Middle_RedPoleBlueNo(s_Swerve, launcherSubsystem, grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  private final DriveFour driveFour = new DriveFour(s_Swerve, launcherSubsystem, conveyorSubsystem);
  private final DriveTen driveTen = new DriveTen(s_Swerve, launcherSubsystem, conveyorSubsystem);
  private final DriveSeven driveSeven = new DriveSeven(s_Swerve, launcherSubsystem, conveyorSubsystem);
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
            m_driver.leftBumper()));
            // leftBumper Sets Driver Orientation Mode

    configureDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();

    // Configure autonomous commands
    configureAutonomousCommands();

  }

  public CameraSubsystem getCameraSubsystem() {
      return cameraSubsystem;
  }

  private void configureDefaultCommands() {
    // Set the default command for rotating the arm
    trapArmSubsystem.setDefaultCommand(
      new RunCommand(() -> {
          double rotateJoystickValue = m_operator.getLeftX();
          double rotateScalingFactor = 0.15;
          trapArmSubsystem.rotate(rotateJoystickValue * rotateScalingFactor);
          double flipJoystickValue = m_operator.getRightX();
          double flipScalingFactor = 0.15;
          trapArmSubsystem.flip(flipJoystickValue * flipScalingFactor);
      }, trapArmSubsystem)
    );
  }

  private void configureAutonomousCommands() {
    autoChooser.setDefaultOption("Shoot Any + Drive 4 Feet", exampleAuto);
    autoChooser.addOption("Side - Side Red No Pole/Side Blue With Pole", side_RedNoBluePole);
    autoChooser.addOption("Side - Side Red with Pole/Side Blue No Pole", side_RedPoleBlueNo);
    autoChooser.addOption("Center - Middle Only", center_Middle);
    autoChooser.addOption("Center - Middle - Side Red with Pole/Side Blue No Pole", center_Middle_RedPoleBlueNo);
    autoChooser.addOption("Center - Middle - Side Red No Pole/Side Blue with Pole", center_Middle_RedNoBluePole);
    autoChooser.addOption("Swerve Drive 4 Feet", driveFour);
    autoChooser.addOption("Swerve Drive 7 Feet", driveSeven);
    autoChooser.addOption("Swerve Drive 10 Feet", driveTen);
    
    // autoChooser.addOption("Launcher Auto", autonomousLauncherCmd);
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

  public GrabberSubsystem getGrabberSubsystem() {
    return grabberSubsystem;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // DON'T FOREGET!!!! The right leftbumper is used to set the Driver Orientation Mode above in Robot Container
    m_driver.rightBumper().whileTrue(new GrabberConveyorReverseCmd(grabberSubsystem, conveyorSubsystem));
    m_driver.rightTrigger(triggerThreshold).whileTrue(new GrabberConveyorCmd(grabberSubsystem, conveyorSubsystem));
    m_driver.y().whileTrue(new ConveyorLauncherCmd(launcherSubsystem, conveyorSubsystem));
    m_driver.a().whileTrue(new ConveyorAmpCmd(launcherSubsystem, conveyorSubsystem));
    m_driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    
    /* Operator Buttons */
    m_operator.leftBumper().whileTrue(new LiftCmd(leftHangerSubsystem, Constants.Mechanisms.upVelocity, 
                                                  Constants.Mechanisms.leftLimitSwitch));
    m_operator.rightBumper().whileTrue(new LiftCmd(rightHangerSubsystem, Constants.Mechanisms.upVelocity, 
                                                   Constants.Mechanisms.rightLimitSwitch));
    m_operator.leftTrigger(triggerThreshold)
        .whileTrue(new LowerCmd(leftHangerSubsystem, Constants.Mechanisms.downVelocity, Constants.Mechanisms.leftLimitSwitch));
    m_operator.rightTrigger(triggerThreshold)
        .whileTrue(new LowerCmd(rightHangerSubsystem, Constants.Mechanisms.downVelocity, Constants.Mechanisms.rightLimitSwitch));
    m_operator.y().whileTrue(new ConveyorLauncherCmd(launcherSubsystem, conveyorSubsystem));
    m_operator.a().whileTrue(new ConveyorAmpCmd(launcherSubsystem, conveyorSubsystem));
    m_operator.b().whileTrue(new TrapGrabCmd2 (trapArmSubsystem));
    m_operator.x().whileTrue(new TrapReleaseCmd2 (trapArmSubsystem));
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
