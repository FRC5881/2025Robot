package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;

// fully extended is 0, retracting is negative
public class ClimberSubsystem extends SubsystemBase {
  public final SparkMax motor = new SparkMax(CANConstants.kClimberId, MotorType.kBrushless);

  public ClimberSubsystem() {
    var config = new SparkMaxConfig();
    config.smartCurrentLimit(30).inverted(false);
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
