package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberSubsystem extends SubsystemBase{
    public CANSparkMax m_GrabberMotor;

    public GrabberSubsystem() {
        m_GrabberMotor = new CANSparkMax(Constants.Mechanisms.grabberMotorID, MotorType.kBrushless);
        m_GrabberMotor.setSmartCurrentLimit(Constants.Mechanisms.grabberCurrentLimit);
    }

    public void setGrabberTargetSpeed (double Speed) {
        m_GrabberMotor.set(Speed);
    }
}