package frc.robot.subsystems.armL1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmL1Subsystem extends SubsystemBase {
  private final PIDController pidController = new PIDController(0.024, 0.0035, 0.00015); // Tuned

  public ArmL1IO io;

  // The angle system is the same as the AlgaeArm
  public Rotation2d setpoint = new Rotation2d(Math.toRadians(0));

  public ArmL1Subsystem() {
    if (RobotBase.isSimulation()) {
      io = new ArmL1IOSim();
    } else {
      io = new ArmL1IOReal();
    }

    setDefaultCommand(cDegControl());
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("L1/currentAngle", io.getCurrentAngle().getDegrees());
    SmartDashboard.putNumber("L1/Setpoint", this.setpoint.getDegrees());
    SmartDashboard.putNumber("L1/voltage", io.getVoltage());
    SmartDashboard.putData("L1/L1PID", pidController);
  }

  public Command cSetpoint(Rotation2d setpoint) {
    return Commands.runOnce(() -> this.setpoint = setpoint);
  }

  public void zero() {
    io.zero();
  }

  public Command cDegControl() {
    return runEnd(
            () -> {
              Rotation2d currentPos = io.getCurrentAngle();
              double voltage =
                  pidController.calculate(currentPos.getDegrees(), this.setpoint.getDegrees());
              io.setVoltage(voltage);
            },
            io::stop)
        .withName("cDegControl");
  }
}
