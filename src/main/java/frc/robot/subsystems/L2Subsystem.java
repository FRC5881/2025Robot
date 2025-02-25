package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class L2Subsystem extends SubsystemBase {
    public final SparkMax L2Motor;
    public final boolean Sensor1;
    public final boolean Sensor2;

    public L2Subsystem() {
        L2Motor = new SparkMax(Constants.CANConstants.kL2Id, MotorType.kBrushless);
        Sensor1 = false;
        Sensor2 = false;

        setDefaultCommand(centerCommand());
    }

    public void output(double speed) {
        L2Motor.set(speed);
    }

    public void center(double speed) {
        if (Sensor1) {
            L2Motor.set(speed);
        } else if (Sensor2) {
            L2Motor.set(-speed);
        } else
            L2Motor.set(0);
    }

    public void stop() {
        L2Motor.set(0);
    }

    public Command outputLeftCommand() {
        double speed = Constants.L2Constants.kL2Speed;
        return startEnd(() -> output(speed), () -> stop());
    }

    public Command centerCommand() {
        double speed = Constants.L2Constants.kL2Speed;
        return startEnd(() -> center(speed), () -> stop());
    }

    public Command outputRightCommand() {
        double speed = -Constants.L2Constants.kL2Speed;
        return startEnd(() -> output(speed), () -> stop());
    }
}
