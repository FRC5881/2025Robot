package frc.robot.subsystems.armL1;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.L1Constants;

public class ArmL1IOReal implements ArmL1IO {
  private final SparkMax motorL1 =
      new SparkMax(Constants.CANConstants.kL1Motor, MotorType.kBrushless);
  private final DigitalInput limitSwitch = new DigitalInput(7);

  public ArmL1IOReal() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);
    config.inverted(true);
    config.softLimit.reverseSoftLimit(0);
    config.softLimit.reverseSoftLimitEnabled(true);
    motorL1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double voltage) {
    motorL1.setVoltage(voltage);
  }

  @Override
  public void zero() {
    motorL1.getEncoder().setPosition(0);
  }

  @Override
  public double getVoltage() {
    return motorL1.getAppliedOutput() * motorL1.getBusVoltage();
  }

  @Override
  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(motorL1.getEncoder().getPosition() / L1Constants.kL1Ratio);
  }

  @Override
  public double getTemp() {
    return motorL1.getMotorTemperature();
  }

  @Override
  public double getCurrent() {
    return motorL1.getOutputCurrent();
  }

  @Override
  public boolean getLimit() {
    return limitSwitch.get();
  }
}
