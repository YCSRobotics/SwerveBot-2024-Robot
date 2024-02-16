package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HangerCmd extends Command {
  private final HangerSubsystem hangerSubsystem;
  

  public HangerCmd(HangerSubsystem hangerSubsystem) {
    this.hangerSubsystem = hangerSubsystem;

    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hangerSubsystem.setHangerPositionTarget(Constants.Mechanisms.hangerTargetSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangerSubsystem.setHangerPositionTarget(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
