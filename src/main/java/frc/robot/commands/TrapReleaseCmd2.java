package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapReleaseCmd2 extends InstantCommand {
  private final TrapArmSubsystem trapArmSubsystem;
  // private final Compressor compressor;

  public TrapReleaseCmd2 (TrapArmSubsystem trapArmSubsystem) {
    this.trapArmSubsystem = trapArmSubsystem;
    // this.compressor = compressor;
      
    addRequirements(trapArmSubsystem);
  }

 
  @Override
  public void initialize() {
    trapArmSubsystem.release();
    // compressor.disable();
  }

  
  @Override
  public void execute() {
    // trapArmSubsystem.release();
  }

  
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
