package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Utils.Constants;

public class ArmIOReal implements ArmIO{
    SparkMax pivotMotor = new SparkMax(Constants.CANConstants.kPivotMotor, MotorType.kBrushless);
    SparkMax intakeMotor = new SparkMax(Constants.CANConstants.kIntakeMotor, MotorType.kBrushless);
    AnalogInput algaeSensor = new AnalogInput(Constants.AnalogInputConstants.kAlgaeSensor);
    
    public ArmIOReal() {
    }

    @Override
    public void setArmVoltages(double pivotVoltage, double intakeVoltage){
        pivotMotor.setVoltage(pivotVoltage);
        intakeMotor.setVoltage(intakeVoltage);
    }

    @Override
    public boolean hasAlgae(){
        //TODO: Make sure this value is right. I got this from 2024
        return algaeSensor.getVoltage()>2.5;
    }

    @Override
    public Rotation2d currentAngle(){
        Rotation2d angle = Rotation2d.fromRotations(pivotMotor.getEncoder().getPosition());
        return angle;
    }

    /*
     * This is in RPM
     */
    @Override
    public double intakeSpeed(){
        double speed = intakeMotor.getEncoder().getVelocity();
        return speed;
    }
}
