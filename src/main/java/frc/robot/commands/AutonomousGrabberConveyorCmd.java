package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ProximitySensorSubsystem;

public class AutonomousGrabberConveyorCmd extends Command {

  private final GrabberSubsystem grabberSubsystem;
  private final double grabberTargetSpeed;
  private final ConveyorSubsystem conveyorSubsystem;
  private final double conveyorTargetSpeed;
  // private final ProximitySensorSubsystem proximitySensorSubsystem;
  private final double durationSeconds;
  public static double startTime;

  /** Creates a new AutonomousCmd. */
  public AutonomousGrabberConveyorCmd(GrabberSubsystem grabberSubsystem, double grabberTargetSpeed, 
                                      ConveyorSubsystem conveyorSubsystem, double conveyorTargetSpeed, 
                                      /*ProximitySensorSubsystem proximitySensorSubsystem, */double durationSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grabberSubsystem = grabberSubsystem;
    this.grabberTargetSpeed = grabberTargetSpeed;
    this.conveyorSubsystem = conveyorSubsystem;
    this.conveyorTargetSpeed = conveyorTargetSpeed;
    // this.proximitySensorSubsystem = proximitySensorSubsystem;
    this.durationSeconds = durationSeconds;
    addRequirements(grabberSubsystem, conveyorSubsystem/*, proximitySensorSubsystem*/);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    grabberSubsystem.setGrabberTargetSpeed(grabberTargetSpeed);
    conveyorSubsystem.setConveyorTargetSpeed(conveyorTargetSpeed);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // launcherSubsystem.setLauncherVelocityTarget(launcherTargetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabberSubsystem.setGrabberTargetSpeed(0.0);
    conveyorSubsystem.setConveyorTargetSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Timer.getFPGATimestamp() - startTime >= durationSeconds || proximitySensorSubsystem.isFieldElementInPosition();
    return Timer.getFPGATimestamp() - startTime >= durationSeconds;
  }
}
