package frc.lib.config;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    //swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360; phoenix 5 code
    swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    //swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert; phoenix 5 code
    swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.valueof(Constants.Swerve.canCoderInvert);
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    
  }
}
