package frc.robot.subsystems.algaeArm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.Optional;

public class AlgaeArmIOReal implements AlgaeArmIO {
  SparkMax pivotMotor = new SparkMax(Constants.CANConstants.kPivotMotor, MotorType.kBrushless);
  SparkMax intakeMotor = new SparkMax(Constants.CANConstants.kIntakeMotor, MotorType.kBrushless);

  public AlgaeArmIOReal() {}

  @Override
  public void setArmVoltages(Optional<Double> pivotVoltage, Optional<Double> intakeVoltage) {
    if (pivotVoltage.isPresent() == true) {
      pivotMotor.setVoltage(pivotVoltage.get());
    }
    if (intakeVoltage.isPresent() == true) {
      intakeMotor.setVoltage(intakeVoltage.get());
    }
  }

  @Override
  public void setArmVoltages(double pivotVoltage, double intakeVoltage) {
    pivotMotor.setVoltage(pivotVoltage);
    intakeMotor.setVoltage(intakeVoltage);
  }

  public double getPivotVoltage() {
    return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
  }

  public double getIntakeVoltage() {
    return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  @Override
  public Rotation2d getCurrentAngle() {
    Rotation2d angle = Rotation2d.fromRotations(pivotMotor.getEncoder().getPosition() / 25);
    return angle;
  }

  @Override
  public double getIntakeSpeed() {
    double speed = intakeMotor.getEncoder().getVelocity();
    return speed;
  }
}
