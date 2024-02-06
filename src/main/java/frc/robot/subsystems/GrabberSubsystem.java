package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase{
    public CANSparkMax m_GrabberMotor;

    public GrabberSubsystem(){
        m_GrabberMotor = new CANSparkMax(11, MotorType.kBrushed);
        m_GrabberMotor.setSmartCurrentLimit(30);
    }

    public void setGrabberTargetSpeed (double Speed){
        m_GrabberMotor.set(Speed);
    }
}