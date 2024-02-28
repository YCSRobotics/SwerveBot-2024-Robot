package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkBase.ControlType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrapArmSubsystem extends SubsystemBase {
  private final DoubleSolenoid pneumaticCylinder;
  private final CANSparkMax flipMotor;
  private final CANSparkMax rotateMotor;
  private final SparkPIDController flipMotorPIDController = flipMotor.getPIDController();

  
  public TrapArmSubsystem() {
    pneumaticCylinder = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    flipMotor = new CANSparkMax(FLIP_MOTOR_ID, MotorType.kBrushless);
    rotateMotor = new CANSparkMax(ROTATE_MOTOR_ID, MotorType.kBrushless);
    flipMotorPIDController = flipMotor.getPIDController();
    
    flipMotorPIDController.setP(PID_P);
    flipMotorPIDController.setI(PID_I);
    flipMotorPIDController.setD(PID_D);
  }

  public void grab() {
    pneumaticCylinder.set(DoubleSolenoid.Value.kForward);
  }

  public void release() {
    pneumaticCylinder.set(DoubleSolenoid.Value.kReverse);
  }

  public void flipToPosition(double position) {
    flipMotorPIDController.setReference(position, ControlType.kPosition);
  }

  public void rotate(double speed) {
    rotateMotor.set(speed);
  }
  
}
