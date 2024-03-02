package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TrapArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;



public class TrapCmd extends SequentialCommandGroup {

  public TrapCmd(TrapArmSubsystem trapArm, double flipPosition, double rotateSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new InstantCommand(() -> trapArm.grab(), trapArm),
      new TrapFlipLeftCmd(trapArm, flipPosition),
      new StartEndCommand(
        () -> trapArm.rotate(rotateSpeed),
        () -> trapArm.rotate(0),
        trapArm
      )
    );
  } 
}
