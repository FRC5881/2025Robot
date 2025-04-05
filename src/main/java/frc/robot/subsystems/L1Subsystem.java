package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class L1Subsystem {
    private final SparkMax motorL1 = new SparkMax(Constants.CANConstants.kL1Motor, MotorType.kBrushless);
    private final DigitalInput limitSwitch = new DigitalInput(7);

    public L1Subsystem() {
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(30);
        config.inverted(true);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(true);
        motorL1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
