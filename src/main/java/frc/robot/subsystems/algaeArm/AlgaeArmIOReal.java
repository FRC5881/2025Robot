package frc.robot.subsystems.algaeArm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import frc.robot.Utils.Constants;

public class AlgaeArmIOReal implements AlgaeArmIO{
    SparkMax pivotMotor = new SparkMax(Constants.CANConstants.kPivotMotor, MotorType.kBrushless);
    SparkMax intakeMotor = new SparkMax(Constants.CANConstants.kIntakeMotor, MotorType.kBrushless);
    
    AnalogInput algaeSensor = new AnalogInput(Constants.AnalogInputConstants.kAlgaeSensor);
    
    public AlgaeArmIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(20);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setArmVoltages(Optional<Double> pivotVoltage, Optional<Double> intakeVoltage){
        if (pivotVoltage.isPresent() == true){
            pivotMotor.setVoltage(pivotVoltage.get());
        }
        if (intakeVoltage.isPresent() == true){
            intakeMotor.setVoltage(intakeVoltage.get());
        }
    }

    @Override
    public void setArmVoltages(double pivotVoltage, double intakeVoltage){
        pivotMotor.setVoltage(pivotVoltage);
        intakeMotor.setVoltage(intakeVoltage);
    }

    public double getPivotVoltage(){
        return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    }

    public double getIntakeVoltage(){
        return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    }

    @Override
    public boolean hasAlgae(){
        //TODO: Make sure this value is right. I got this from 2024
        return algaeSensor.getVoltage()>2.5;
    }

    @Override
    public Rotation2d getCurrentAngle(){
        //10 degrees of slop
        Rotation2d angle = Rotation2d.fromRotations(pivotMotor.getEncoder().getPosition()/5).plus(Rotation2d.fromDegrees(Constants.PositionConstants.kAlgaeDegreeOffset));
        return angle;
    }

    /*
     * This is in RPM
     */
    @Override
    public double getIntakeSpeed(){
        double speed = intakeMotor.getEncoder().getVelocity();
        return speed;
    }

}
