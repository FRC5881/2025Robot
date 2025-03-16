package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeArmSubsystem extends SubsystemBase {
  private final ArmFeedforward pivotFF = new ArmFeedforward(0, 0.9, 0);
  private final PIDController pivotPID = new PIDController(0.3, 0, 0);

  private static final double MAX_VOLTAGE = 12.0;

  public AlgaeArmIO io;

  private Rotation2d setpoint = Constants.ArmConstants.kSTART;
  private double intakeVoltage = 0; // This is in RPM

  public AlgaeArmSubsystem() {
    if (RobotBase.isSimulation()) {
      io = new AlgaeArmIOSim();
    } else {
      io = new AlgaeArmIOReal();
    }
    setDefaultCommand(cControl());

    SmartDashboard.putData("Arm/pivotPID", pivotPID);
  }

  @Override
  public void simulationPeriodic() {
    ((AlgaeArmIOSim) io).pivotSim.update(0.020);
    ((AlgaeArmIOSim) io).intakeSim.update(0.020);
  }

  @Override
  public void periodic() {
    // Smart Dashboard Intake
    SmartDashboard.putNumber("Arm/intakeSpeed", io.getIntakeSpeed());
    SmartDashboard.putNumber("Arm/intakeVoltage", io.getIntakeVoltage());

    // Smart Dashboard Pivot
  }

  public void zero() {
    this.io.zero();
  }

  public void set(Rotation2d setpoint, double intakeVoltage) {
    this.intakeVoltage = intakeVoltage;
    this.setpoint = setpoint;
  }

  public void setpoint(Rotation2d setpoint) {
    this.setpoint = setpoint;
  }

  public void intake(double intakeVoltage) {
    this.intakeVoltage = intakeVoltage;
  }

  private Command cControl() {
    return runEnd(
        () -> {
          Rotation2d current = io.getCurrentAngle();
          double pivotVoltage =
              pivotFF.calculate(setpoint.getRadians(), 0)
                  + pivotPID.calculate(current.getDegrees(), setpoint.getDegrees());
          pivotVoltage = MathUtil.clamp(pivotVoltage, -MAX_VOLTAGE, +MAX_VOLTAGE);
          io.setArmVoltages(pivotVoltage, intakeVoltage);

          SmartDashboard.putNumber("Arm/pivotDegree", current.getDegrees());
          SmartDashboard.putNumber("Arm/pivotVoltage", pivotVoltage);
          SmartDashboard.putNumber("Arm/truePivotSetpoint", setpoint.getDegrees());
        },
        io::stop);
  }
}
