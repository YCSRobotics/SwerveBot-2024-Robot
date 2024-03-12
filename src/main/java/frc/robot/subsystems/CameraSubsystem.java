package frc.robot.subsystems;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CameraSubsystem extends SubsystemBase {
  private UsbCamera camera1;
  //private UsbCamera camera2;

  /** Creates a new ProximitySensorSubsystem. */
  public CameraSubsystem() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera1.setResolution(160, 120);
    camera1.setFPS(15);
    //camera2 = CameraServer.startAutomaticCapture(1);
    //camera2.setResolution(160, 120);
    //camera2.setFPS(15);
  }

  @Override
  public void periodic() {
  }

}
