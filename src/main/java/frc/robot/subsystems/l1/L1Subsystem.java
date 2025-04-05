package frc.robot.subsystems.l1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class L1Subsystem extends SubsystemBase {
  private final PIDController pidController = new PIDController(0.075, 0, 0); // Tuned
  private final SlewRateLimiter limit = new SlewRateLimiter(110 / 0.15, -110.0 / 0.3, 0.0);

  public L1IO io;

  // The angle system is the same as the AlgaeArm
  public Rotation2d setpoint = new Rotation2d(Math.toRadians(0));

  public L1Subsystem() {
    if (RobotBase.isSimulation()) {
      io = new L1IOSim();
    } else {
      io = new L1IOReal();
    }

    pidController.setTolerance(2.5);
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
    SmartDashboard.putNumber("L1/current", io.getCurrent());
    SmartDashboard.putNumber("L1/temp", io.getTemp());
    SmartDashboard.putBoolean("L1/limit", io.getLimit());
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
              double setpoint = limit.calculate(this.setpoint.getDegrees());
              Rotation2d currentPos = io.getCurrentAngle();
              double voltage = pidController.calculate(currentPos.getDegrees(), setpoint);
              if (pidController.atSetpoint()) {
                io.setVoltage(0);
              } else {
                io.setVoltage(voltage);
              }
            },
            io::stop)
        .withName("cDegControl");
  }
}
