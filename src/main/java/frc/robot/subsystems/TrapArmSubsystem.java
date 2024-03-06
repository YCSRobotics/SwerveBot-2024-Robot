package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class TrapArmSubsystem extends SubsystemBase {
  
  private Solenoid trapArmSolenoid;
  private final CANSparkMax flipMotor;
  private final CANSparkMax rotateMotor;

  private final RelativeEncoder rotateEncoder;
  private final RelativeEncoder flipEncoder;

  private final PneumaticHub m_ph;

  
  public TrapArmSubsystem() {
    flipMotor = new CANSparkMax(Constants.Mechanisms.flipMotorID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(Constants.Mechanisms.rotateMotorID, MotorType.kBrushless);
    
    trapArmSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Mechanisms.solenoidPort);

    m_ph = new PneumaticHub(1);
    m_ph.enableCompressorAnalog(100,120);
    
    rotateEncoder = rotateMotor.getEncoder();
    flipEncoder = flipMotor.getEncoder();
  }

  
  public void grab(){
    trapArmSolenoid.set (false);
    m_ph.enableCompressorAnalog(100,120);
  }
  
  public void release(){
    trapArmSolenoid.set (true);
    //m_ph.enableCompressorAnalog(0,0);
    m_ph.disableCompressor();
  }
  
  public void flip (double speed){
    double currentPosition = flipEncoder.getPosition();
    // Check if moving within bounds
    if ((speed > 0 && currentPosition < Constants.Mechanisms.flipMaxLimit) || 
        (speed < 0 && currentPosition > Constants.Mechanisms.flipMinLimit)) {
        flipMotor.set(speed);
    } else {
        flipMotor.set(0);
    }
  }

  public void rotate(double speed) {
    double currentPosition = rotateEncoder.getPosition();
    // Check if moving within bounds
    if ((speed > 0 && currentPosition < Constants.Mechanisms.rotateMaxLimit) || 
        (speed < 0 && currentPosition > Constants.Mechanisms.rotateMinLimit)) {
        rotateMotor.set(speed);
    } else {
        rotateMotor.set(0);
    }
  }
//A method to reset the encoder to zero
  public void resetRotateEncoder() {
    rotateEncoder.setPosition(0);
  }
}