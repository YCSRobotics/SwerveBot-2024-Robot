package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ProximitySensorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GrabberConveyorCmd extends Command {
  private final GrabberSubsystem grabberSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private final ProximitySensorSubsystem proximitySensorSubsystem;

  /** Creates a new GrabberConveyorCmd. */
  public GrabberConveyorCmd(GrabberSubsystem grabberSubsystem, ConveyorSubsystem conveyorSubsystem, ProximitySensorSubsystem proximitySensorSubsystem) {
    this.grabberSubsystem = grabberSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.proximitySensorSubsystem = proximitySensorSubsystem;
      
    addRequirements(grabberSubsystem, conveyorSubsystem, proximitySensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   //if (operator.getRawButton(1) && !proximitySensorSubsystem.isFieldElementInPosition()) { 
   //while (!proximitySensorSubsystem.isFieldElementInPosition()) { 
      SmartDashboard.putNumber("Proximity Sensor Cmd", proximitySensorSubsystem.proximitySensor.getProximity());
      SmartDashboard.putBoolean("ElementInPosition Cmd", proximitySensorSubsystem.isFieldElementInPosition());
      SmartDashboard.putBoolean("isFinished", isFinished());
      
      grabberSubsystem.setGrabberTargetSpeed(Constants.Mechanisms.grabberTargetSpeed);
      conveyorSubsystem.setConveyorTargetSpeed(Constants.Mechanisms.conveyorTargetSpeed);
   //}

     //} else {
     //  grabberSubsystem.setGrabberTargetSpeed(0);
     //  conveyorSubsystem.setConveyorTargetSpeed(0);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabberSubsystem.setGrabberTargetSpeed(0);
    conveyorSubsystem.setConveyorTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (operator.getRawButtonReleased(1) || proximitySensorSubsystem.isFieldElementInPosition()) {
    //   return true;
    // } else {
    //   return false;
    // }

    return proximitySensorSubsystem.isFieldElementInPosition();
  }
}