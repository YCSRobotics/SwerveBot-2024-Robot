package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase{
    public CANSparkMax m_ConveyorMotor;

    public ConveyorSubsystem() {
        m_ConveyorMotor = new CANSparkMax(Constants.Mechanisms.conveyorMotorID, MotorType.kBrushless);
        m_ConveyorMotor.setSmartCurrentLimit(Constants.Mechanisms.conveyorCurrentLimit);
    }

    public void setConveyorTargetSpeed (double Speed) {
        m_ConveyorMotor.set(-Speed);
    }
}