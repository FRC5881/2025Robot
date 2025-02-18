package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class L1Subsystem extends SubsystemBase {
    private final PWMSparkMax motor;
    private final Encoder encoder;

    public L1Subsystem() {
        motor = new PWMSparkMax(0);
        encoder = new Encoder(0, 1);
    }

    public void moveToLowerPosition() {
        while (encoder.getDistance() > Constants.L1Constants.kLowerPosition + Constants.L1Constants.kTolerance) {
            motor.set(-0.5);
        }
        motor.stopMotor();
    }

    public void moveToUpperPosition() {
        while (encoder.getDistance() < Constants.L1Constants.kUpperPosition - Constants.L1Constants.kTolerance) {
            motor.set(0.5);
        }
        motor.stopMotor();
    }

    public void stop() {
        motor.stopMotor();
    }

    public Command moveToLowerPositionCommand() {
        return runEnd(() -> moveToLowerPosition(), () -> stop());
    };

    public Command moveToUpperPositionCommand() {
        return runEnd(() -> moveToUpperPosition(), () -> stop());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
