package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase{
    public CANSparkMax m_ConveyorMotor;

    public ConveyorSubsystem(){
        m_ConveyorMotor = new CANSparkMax(12, MotorType.kBrushed);
        m_ConveyorMotor.setSmartCurrentLimit(30);
    }

    public void setConveyorTargetSpeed (double Speed){
        m_ConveyorMotor.set(Speed);
    }
}