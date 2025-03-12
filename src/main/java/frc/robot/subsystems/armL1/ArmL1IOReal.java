package frc.robot.subsystems.armL1;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmL1IOReal implements ArmL1IO {
  SparkMax motorL1 = new SparkMax(Constants.CANConstants.kMotorL1, MotorType.kBrushless);

  @Override
  public void setVoltage(double voltage) {
    motorL1.setVoltage(voltage);
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
