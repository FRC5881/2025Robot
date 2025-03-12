package frc.robot.subsystems.algaeArm;

import java.util.Optional;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArmSubsystem extends SubsystemBase {
    // TODO: Make proper new values
    private final ArmFeedforward pivotFF = new ArmFeedforward(0, 0.33, 0);
    private final PIDController pivotPID = new PIDController(0.12, 0, 0); // This is okay but needs more tuning
    private final SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0, 0, 0);
    private final PIDController intakePID = new PIDController(0, 0, 0);

    public AlgaeArmIO io;

    public Rotation2d pivotSetpoint = new Rotation2d(Math.toRadians(81));
    public double algaeIntakeSetpoint = 0; // This is in RPM

    public AlgaeArmSubsystem() {
        if (RobotBase.isSimulation()) {
            io = new AlgaeArmIOSim();
        } else {
            io = new AlgaeArmIOReal();
        }
        setDefaultCommand(cDegControl());
        SmartDashboard.putNumber("Arm/mySetpoint", 23);
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
        SmartDashboard.putData("Arm/intakePID", intakePID);

        // Smart Dashboard Pivot
        SmartDashboard.putNumber("Arm/pivotDegree", io.getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/pivotVoltage", io.getPivotVoltage());
        SmartDashboard.putData("Arm/pivotPID", pivotPID);
        SmartDashboard.putNumber("Arm/truePivotSetpoint", this.pivotSetpoint.getDegrees());
    }

    public Command cDegControl() {
        return runEnd(() -> {
            Rotation2d currentPos = io.getCurrentAngle();
            io.setArmVoltages(
                    Optional.of(pivotFF.calculate(this.pivotSetpoint.getRadians(), 0)
                            + pivotPID.calculate(currentPos.getDegrees(), this.pivotSetpoint.getDegrees())),
                    Optional.of(intakeFF.calculate(this.algaeIntakeSetpoint)
                            + intakePID.calculate(io.getIntakeSpeed(), this.algaeIntakeSetpoint)));
        }, io::stop);

    }
}
