package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    public final SparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(Constants.CANConstants.kIntakeId, MotorType.kBrushless);
    }

    public void intake() {
        // Intake coral
        intakeMotor.set(Constants.IntakeConstants.kIntakeSpeed);
    }

    public void stop() {
        // Stop intake
        intakeMotor.stopMotor();
    }

    public void reverse() {
        // Reverse intake
        intakeMotor.set(-Constants.IntakeConstants.kIntakeSpeed);
    }

}
