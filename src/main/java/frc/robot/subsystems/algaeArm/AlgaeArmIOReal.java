package frc.robot.subsystems.algaeArm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AlgaeArmIOReal implements AlgaeArmIO {
  SparkMax pivotMotor = new SparkMax(Constants.CANConstants.kPivotMotor, MotorType.kBrushless);
  SparkMax intakeMotor = new SparkMax(Constants.CANConstants.kIntakeMotor, MotorType.kBrushless);

  public AlgaeArmIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.smartCurrentLimit(25);
    config.idleMode(IdleMode.kBrake);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig config2 = new SparkMaxConfig();
    config.inverted(true);
    config.smartCurrentLimit(25);
    intakeMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setArmVoltages(double pivotVoltage, double intakeVoltage) {
    // pivotMotor.setVoltage(pivotVoltage);
    intakeMotor.setVoltage(intakeVoltage);
    SmartDashboard.putNumber("Arm/Pivot Current", pivotMotor.getOutputCurrent());
  }

  @Override
  public void zero() {
    pivotMotor.getEncoder().setPosition(0);
  }

  public double getPivotVoltage() {
    return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
  }

  public double getIntakeVoltage() {
    return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  @Override
  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(pivotMotor.getEncoder().getPosition() / 12)
        .plus(Constants.ArmConstants.kSTART);
  }

  // Rotations per minute
  @Override
  public double getIntakeSpeed() {
    return intakeMotor.getEncoder().getVelocity();
  }
}
