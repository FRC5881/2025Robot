package frc.robot.subsystems.armL1;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Utils.Constants;

public class ArmL1IOReal implements ArmL1IO {

    SparkMax motorL1 = new SparkMax(Constants.CANConstants.kMotorL1, MotorType.kBrushless);
    AnalogInput coralSensorL1 = new AnalogInput(Constants.AnalogInputConstants.kCoralSensorL1);



    @Override
    public void setVoltage(double voltage) {
        motorL1.setVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return motorL1.getAppliedOutput() * motorL1.getBusVoltage();
    }

    @Override
    public boolean hasCoral() {
        //TODO: Make sure this value is right. I got this from 2024
        return coralSensorL1.getVoltage()>2.5;
    }

    @Override
    public Rotation2d getCurrentAngle() {
        Rotation2d angle = Rotation2d.fromRotations(motorL1.getEncoder().getPosition());
        return angle;
    }

}
