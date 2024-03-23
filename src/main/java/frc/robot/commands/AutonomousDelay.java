package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousDelay extends Command {
  
  public double delaytime;
  public static double startTime;

  /** Creates a new AutonomousCmd. */
  public AutonomousDelay(double delaytime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.delaytime = delaytime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= delaytime;
  }
}