package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapGrabCmd2 extends InstantCommand {
  private final TrapArmSubsystem trapArmSubsystem;

  public TrapGrabCmd2 (TrapArmSubsystem trapArmSubsystem) {
    this.trapArmSubsystem = trapArmSubsystem;
      
    addRequirements(trapArmSubsystem);
  }

 
  @Override
  public void initialize() {
    trapArmSubsystem.grab();
  }

  
  @Override
  public void execute() {
    // trapArmSubsystem.grab();
  }

  
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
