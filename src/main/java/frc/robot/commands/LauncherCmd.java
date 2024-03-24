package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.Constants;

public class LauncherCmd extends Command {
  private final LauncherSubsystem launcherSubsystem;
  private final double leftLaunchertargetVelocity;
  private final double rightLaunchertargetVelocity;

  /** Creates a new SetlauncherSpeedCmd. */
  public LauncherCmd(LauncherSubsystem launcherSubsystem, double leftLaunchertargetVelocity, double rightLaunchertargetVelocity) {
    this.launcherSubsystem = launcherSubsystem;
    this.leftLaunchertargetVelocity = leftLaunchertargetVelocity;
    this.rightLaunchertargetVelocity = rightLaunchertargetVelocity;

    addRequirements(launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSubsystem.setLauncherVelocityTarget(leftLaunchertargetVelocity, rightLaunchertargetVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSubsystem.setLauncherVelocityTarget(Constants.Mechanisms.leftLauncherVelocityTarget, 
                                                Constants.Mechanisms.rightLauncherVelocityTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.setLauncherVelocityTarget(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
