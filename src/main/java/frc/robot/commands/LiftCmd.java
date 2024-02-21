package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftCmd extends Command {
  private final HangerSubsystem hangerSubsystem;
  private final DigitalInput limitSwitch;
  private final double targetVelocity;

  public LiftCmd(HangerSubsystem hangerSubsystem, int hangerMotorID, double targetVelocity, DigitalInput limitSwitch) {
    this.hangerSubsystem = hangerSubsystem;
    this.limitSwitch = limitSwitch;
    this.targetVelocity = targetVelocity;

    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hangerSubsystem.setHangerVelocityTarget(targetVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangerSubsystem.setHangerVelocityTarget(targetVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangerSubsystem.setHangerVelocityTarget(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
