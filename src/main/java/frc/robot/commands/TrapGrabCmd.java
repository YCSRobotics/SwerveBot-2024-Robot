package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapGrabCmd extends InstantCommand {
  public TrapGrabCmd (TrapArmSubsystem trapArmSubsystem) {
    super(trapArmSubsystem::grab, trapArmSubsystem);
  }

 
  @Override
  public void initialize() {
    
  }

  
  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
