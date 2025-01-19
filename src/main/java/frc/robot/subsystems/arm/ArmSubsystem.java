package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // TODO: Make proper new values
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0,0,0);
    private final PIDController pidController = new PIDController(0, 0, 0);
    private ArmIO io;

    public ArmSubsystem() {
        if (RobotBase.isSimulation()) {
            // io = new ArmIOSim();
            throw new RuntimeException("The simulated arm doesn't exist yet");
        } else {
            // io = new ArmIOReal();
            throw new RuntimeException("The real arm doesn't exist yet");
        }
    }

    public Command cPivotDegControl(Rotation2d setpoint){
        return runEnd(() -> {
            Rotation2d currentPos = io.currentAngle();

            io.setArmVoltages(feedforward.calculate(setpoint.getDegrees()) + pidController.calculate(currentPos.getDegrees(), setpoint.getDegrees()), 0);
        }, io::stop);
    }
}
