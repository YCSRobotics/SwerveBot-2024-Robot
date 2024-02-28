package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapCmd extends SequentialCommandGroup {

  public TrapCmd(TrapArmSubsystem trapArm, double flipToPosition, double rotateSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new InstantCommand(() -> trapArm.grab(), trapArm),
      new FlipCommand(trapArm, flipToPosition),
      new StartEndCommand(
        () -> trapArm.rotate(rotateSpeed),
        () -> trapArm.rotate(0),
        trapArm
      )
    );

  }
  
}
