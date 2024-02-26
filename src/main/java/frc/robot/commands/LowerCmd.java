package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LowerCmd extends Command {
  private final HangerSubsystem hangerSubsystem;
  private final double targetVelocity;

  public LowerCmd(HangerSubsystem hangerSubsystem, double targetVelocity, DigitalInput limitSwitch) {
    this.hangerSubsystem = hangerSubsystem;
    this.targetVelocity = targetVelocity;
    
    addRequirements(hangerSubsystem);
  }

  // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   hangerSubsystem.setHangerVelocityTarget(targetVelocity);
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Target Hanger Velocity", targetVelocity);
    hangerSubsystem.setHangerVelocityTarget(targetVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangerSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
