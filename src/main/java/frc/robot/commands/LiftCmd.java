package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftCmd extends Command {
  private final HangerSubsystem hangerSubsystem;
  private Joystick operator;
  

  public LiftCmd(HangerSubsystem hangerSubsystem, Joystick operator) {
    this.hangerSubsystem = hangerSubsystem;
    this.operator = operator;

    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hangerSubsystem.setHangerPositionTarget(Constants.Mechanisms.hangerTargetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangerSubsystem.setHangerPositionTarget(Constants.Mechanisms.hangerTargetPosition);
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
