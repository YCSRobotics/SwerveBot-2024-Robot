package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class TrapArmSubsystem extends SubsystemBase {
  //private final DoubleSolenoid pneumaticCylinder;
  private Solenoid trapArmSolenoid;
  private final CANSparkMax flipMotor;
  private final CANSparkMax rotateMotor;
  //private final SparkPIDController flipMotorPIDController;

  
  public TrapArmSubsystem() {
    //pneumaticCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
     
    flipMotor = new CANSparkMax(Constants.Mechanisms.flipMotorID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(Constants.Mechanisms.rotateMotorID, MotorType.kBrushless);
    trapArmSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Mechanisms.solenoidPort);
    //flipMotorPIDController = flipMotor.getPIDController();
    
    //flipMotorPIDController.setP(Constants.Mechanisms.trapArmPIDControllerkP);
    //flipMotorPIDController.setI(Constants.Mechanisms.trapArmPIDControllerkI);
    //flipMotorPIDController.setD(Constants.Mechanisms.trapArmPIDControllerkD);

  }

  //public void grab() {
  // pneumaticCylinder.set(DoubleSolenoid.Value.kForward);
  //}

  // public void release() {
  //   pneumaticCylinder.set(DoubleSolenoid.Value.kReverse);
  // }


  //public void flipToPosition(double position) {
    //flipMotorPIDController.setReference(position, ControlType.kPosition);
  //}
  public void grab(){
    trapArmSolenoid.set (true);
  }
  
  public void release(){
    trapArmSolenoid.set (false);
  }
  public void flip (double speed){
    if ((Math.abs(flipMotor.getEncoder().getPosition())) < Constants.Mechanisms.flipLimit) {
      rotateMotor.set(speed);
    } else {
      rotateMotor.set(0);
    }
  }

  public void rotate(double speed) {
    if ((Math.abs(rotateMotor.getEncoder().getPosition())) < Constants.Mechanisms.rotateLimit) {
      rotateMotor.set(speed);
    } else {
      rotateMotor.set(0);
    }
  }

}