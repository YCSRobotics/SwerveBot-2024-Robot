package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ProximitySensorSubsystem;

public class ConveyorLauncherCmd extends Command {
  private final ConveyorSubsystem conveyorSubsystem;
  private final LauncherSubsystem launcherSubsystem;
  private Joystick operator;
  private ProximitySensorSubsystem proximitySensorSubsystem;

  /** Creates a new ConveyorLauncherCmd. */
  public ConveyorLauncherCmd(LauncherSubsystem launcherSubsystem, ConveyorSubsystem conveyorSubsystem, Joystick operator, ProximitySensorSubsystem proximitySensorSubsystem) {
    this.launcherSubsystem = launcherSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.operator = operator;
    this.proximitySensorSubsystem = proximitySensorSubsystem;

    addRequirements(launcherSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operator.getRawButtonPressed(2)) {
      launcherSubsystem.setLauncherTargetSpeed(Constants.Mechanisms.launcherTargetSpeed);
      conveyorSubsystem.setConveyorTargetSpeed(Constants.Mechanisms.conveyorTargetSpeed);

    // } else {
    //   launcherSubsystem.setLauncherTargetSpeed(0);
    //   conveyorSubsystem.setConveyorTargetSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.setLauncherTargetSpeed(0);
    conveyorSubsystem.setConveyorTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isFieldElementInPosition() {
    return proximitySensorSubsystem.isFieldElementInPosition();
  }
}
