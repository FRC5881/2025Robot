package frc.robot.subsystems.armL1;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmL1IOReal implements ArmL1IO {
  SparkMax motorL1 = new SparkMax(Constants.CANConstants.kL1Motor, MotorType.kBrushless);

  @Override
  public void setVoltage(double voltage) {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);
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
    return Rotation2d.fromRotations(motorL1.getEncoder().getPosition() / 16);
  }
}
