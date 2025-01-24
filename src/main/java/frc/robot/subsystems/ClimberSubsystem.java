package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    public final SparkMax climberMotor;
    public final SparkMax climberMotorFollower;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(Constants.CANConstants.kClimberId, MotorType.kBrushless);
        climberMotorFollower = new SparkMax(Constants.CANConstants.kClimberFollowerId, MotorType.kBrushless);

    }

    public void setSpeed(double speed) {
        climberMotor.set(speed);
        climberMotorFollower.set(speed);
    }

    public void stop() {
        climberMotor.stopMotor();
        climberMotorFollower.stopMotor();
    }

    public void climbUp(double speed) {
        climberMotor.setVoltage(speed);
        climberMotorFollower.setVoltage(speed);
    }

    public double getTorque() {
        return climberMotor.getOutputCurrent() * Constants.ClimberConstants.kMaxTorque;
    }

    public Command climbUpCommand() {
        return new Command() {
            @Override
            public void initialize() {
                double torque = getTorque();
                double speed = calculateSpeed(torque);
                climbUp(speed);
            }

            @Override
            public void execute() {
                // Add any additional execution logic here if needed
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }

            @Override
            public boolean isFinished() {
                return false; // Change this condition as per your requirement
            }
        };
    }

    private double calculateSpeed(double torque) {
        double maxTorque = Constants.ClimberConstants.kMaxTorque;
        double minSpeed = Constants.ClimberConstants.kMinClimberSpeed;
        double maxSpeed = Constants.ClimberConstants.kClimberSpeed;

        if (torque >= maxTorque) {
            return minSpeed;
        } else {
            return (torque / maxTorque) * (maxSpeed - minSpeed) + minSpeed;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}