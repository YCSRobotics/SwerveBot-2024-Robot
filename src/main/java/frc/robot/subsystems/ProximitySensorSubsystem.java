package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ProximitySensorSubsystem extends SubsystemBase {
  public final AnalogInput proximitySensor = new AnalogInput(0);
  /** Creates a new ProximitySensorSubsystem. */
  public ProximitySensorSubsystem() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Sensor Average Voltage", proximitySensor.getAverageVoltage());
    //SmartDashboard.putBoolean("ElementInPosition", isFieldElementInPosition());
  }
  public boolean isFieldElementInPosition() {
    return proximitySensor.getAverageVoltage() > (3);
  }
}