package frc.robot.commands;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Joystick;


public class GrabberConveyorCmd extends Command {
  private final GrabberSubsystem grabberSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private Joystick operator = new Joystick(1);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 proximitySensor = new ColorSensorV3(i2cPort);

  private static final double grabberTargetSpeed = 0.5;
  private static final double conveyorTargetSpeed = 0.5;


  /** Creates a new GrabberConveyorCmd. */
  public GrabberConveyorCmd(GrabberSubsystem grabberSubsystem, ConveyorSubsystem conveyorSubsystem, Joystick operator) {
    this.grabberSubsystem = grabberSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.operator = operator;
      
    addRequirements(grabberSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operator.getRawButtonPressed(1) && !isFieldElementInPosition()) {
      grabberSubsystem.setGrabberTargetSpeed(Constants.Mechanisms.grabberTargetSpeed);
      conveyorSubsystem.setConveyorTargetSpeed(Constants.Mechanisms.conveyorTargetSpeed);

    } else {
      grabberSubsystem.setGrabberTargetSpeed(0);
      conveyorSubsystem.setConveyorTargetSpeed(0);
    }
  }

  public boolean isFieldElementInPosition() {
    return proximitySensor.getProximity() > (1500);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
