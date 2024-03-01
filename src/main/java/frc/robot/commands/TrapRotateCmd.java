package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapRotateCmd extends Command {
  private final TrapArmSubsystem trapArmSubsystem;
  private final double speed;

  public TrapRotateCmd (TrapArmSubsystem trapArmSubsystem, double speed) {
    this.trapArmSubsystem = trapArmSubsystem;
    this.speed = speed;
    addRequirements(trapArmSubsystem);
  }

 
  @Override
  public void initialize() {
    trapArmSubsystem.rotate(speed);
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
