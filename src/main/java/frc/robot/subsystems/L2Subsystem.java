package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// + voltage = exits left
public class L2Subsystem extends SubsystemBase {
  public final SparkMax conveyor = new SparkMax(Constants.CANConstants.kL2Id, MotorType.kBrushless);
  public final SparkMax ramp = new SparkMax(Constants.CANConstants.kL2RampId, MotorType.kBrushless);

  public final DigitalInput reefSensor = new DigitalInput(8);
  public final DigitalInput coralSensor = new DigitalInput(9);

  public L2Subsystem() {
    var conveyorConfig = new SparkMaxConfig();
    conveyorConfig.smartCurrentLimit(20);
    conveyor.configure(
        conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var rampConfig = new SparkMaxConfig();
    rampConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    rampConfig.encoder.positionConversionFactor(1.0 / (5 * 5));
    rampConfig.closedLoop.outputRange(-6, 6).p(1.0);
    ramp.configure(rampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
