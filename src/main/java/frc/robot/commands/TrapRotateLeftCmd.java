package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapRotateLeftCmd extends Command {
  private final TrapArmSubsystem trapArmSubsystem;

  public TrapRotateLeftCmd (TrapArmSubsystem trapArmSubsystem) {
    this.trapArmSubsystem = trapArmSubsystem;
    addRequirements(trapArmSubsystem);
  }

 
  @Override
  public void initialize() {
    trapArmSubsystem.rotate(Constants.Mechanisms.trapArmRotateLeftSpeed);
  }

  
  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {
    trapArmSubsystem.rotate(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
