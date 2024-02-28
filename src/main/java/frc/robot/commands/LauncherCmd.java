package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.Constants;

public class LauncherCmd extends Command {
  private final LauncherSubsystem launcherSubsystem;
  private final double targetVelocity;
  

  /** Creates a new SetlauncherSpeedCmd. */
  public LauncherCmd(LauncherSubsystem launcherSubsystem, double targetVelocity) {
    this.launcherSubsystem = launcherSubsystem;
    this.targetVelocity = targetVelocity;
    
    addRequirements(launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSubsystem.setLauncherVelocityTarget(targetVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSubsystem.setLauncherVelocityTarget(Constants.Mechanisms.launcherVelocityTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.setLauncherVelocityTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
