package frc.robot.subsystems;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProximitySensorSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 proximitySensor = new ColorSensorV3(i2cPort);

  /** Creates a new ProximitySensorSubsystem. */
  public ProximitySensorSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Proximity Sensor", proximitySensor.getProximity());
    SmartDashboard.putBoolean("ElementInPosition", isFieldElementInPosition());
  }

  public boolean isFieldElementInPosition() {
    return proximitySensor.getProximity() > (1500);
  }
}
