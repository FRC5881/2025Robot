package frc.robot.subsystems.algaeArm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import java.util.Optional;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class AlgaeArmIOReal implements AlgaeArmIO {
    SparkMax pivotMotor = new SparkMax(Constants.CANConstants.kPivotMotor, MotorType.kBrushless);
    SparkMax intakeMotor = new SparkMax(Constants.CANConstants.kIntakeMotor, MotorType.kBrushless);

    public AlgaeArmIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(20);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setArmVoltages(Optional<Double> pivotVoltage, Optional<Double> intakeVoltage) {
        if (pivotVoltage.isPresent()) {
            pivotMotor.setVoltage(pivotVoltage.get());
        }
        if (intakeVoltage.isPresent()) {
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
        return Rotation2d.fromRotations(pivotMotor.getEncoder().getPosition() / 5);
    }

    // Rotations per minute
    @Override
    public double getIntakeSpeed() {
        return intakeMotor.getEncoder().getVelocity();
    }
}
