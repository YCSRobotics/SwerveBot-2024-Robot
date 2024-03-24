package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;

public class AutonomousLauncherCmd extends Command {

  private final LauncherSubsystem launcherSubsystem;
  private final double leftLauncherVelocityTarget;
  private final double rightLaunchertargetVelocity;
  private final ConveyorSubsystem conveyorSubsystem;
  private final double conveyorTargetSpeed;
  private final double durationSeconds;
  public static double startTime;

  /** Creates a new AutonomousCmd. */
  public AutonomousLauncherCmd(LauncherSubsystem launcherSubsystem, double leftLaunchertargetVelocity, double rightLaunchertargetVelocity, ConveyorSubsystem conveyorSubsystem, double conveyorTargetSpeed, double durationSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSubsystem = launcherSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.leftLauncherVelocityTarget = leftLaunchertargetVelocity;
    this.rightLaunchertargetVelocity = rightLaunchertargetVelocity;
    this.conveyorTargetSpeed = conveyorTargetSpeed;
    this.durationSeconds = durationSeconds;
    addRequirements(launcherSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSubsystem.setLauncherVelocityTarget(leftLauncherVelocityTarget, rightLaunchertargetVelocity);
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
    launcherSubsystem.setLauncherVelocityTarget(0.0, 0.0);
    conveyorSubsystem.setConveyorTargetSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= durationSeconds;
  }
}
