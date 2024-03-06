package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ConveyorAmpCmd extends Command {
  private final ConveyorSubsystem conveyorSubsystem;
  private final LauncherSubsystem launcherSubsystem;

  /** Creates a new ConveyorLauncherCmd. */
  public ConveyorAmpCmd(LauncherSubsystem launcherSubsystem, ConveyorSubsystem conveyorSubsystem) {
    this.launcherSubsystem = launcherSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;

    addRequirements(launcherSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      launcherSubsystem.setLauncherVelocityTarget(Constants.Mechanisms.ampVelocityTargetSpeed);
      conveyorSubsystem.setConveyorTargetSpeed(Constants.Mechanisms.conveyorTargetSpeed);

    // } else {
    //   launcherSubsystem.setLauncherVelocityTarget(0);
    //   conveyorSubsystem.setConveyorTargetSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.setLauncherVelocityTarget(0);
    conveyorSubsystem.setConveyorTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isFieldElementInPosition() {
    return false;
  }
}
