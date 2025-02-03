package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // TODO: Make proper new values
    private final SimpleMotorFeedforward pivotFF = new SimpleMotorFeedforward(0,0,0);
    private final PIDController pivotPID = new PIDController(0.15, 0.1, 0.05); //This is okay but needs more tuning
    private final SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0,0,0);
    private final PIDController intakePID = new PIDController(0, 0, 0);

    private ArmIO io;

    public Rotation2d pivotSetpoint = new Rotation2d(Math.toRadians(-90));

    public ArmSubsystem() {
        if (RobotBase.isSimulation()) {
            io = new ArmIOSim();
            
            // throw new RuntimeException("The simulated arm doesn't exist yet");
        } else {
            io = new ArmIOReal();
            // throw new RuntimeException("The real arm doesn't exist yet");
        }
        this.setDefaultCommand(cPivotDegControl());
        SmartDashboard.putNumber("Arm/mySetpoint", 20);
    }
    

    @Override
    public void simulationPeriodic() {
        ((ArmIOSim) io).pivotSim.update(0.020);
        ((ArmIOSim) io).intakeSim.update(0.020);
    }

    @Override
    public void periodic() {
        //Smart Dashboard Intake
        SmartDashboard.putNumber("Arm/intakeSpeed", io.getIntakeSpeed());
        SmartDashboard.putNumber("Arm/intakeVoltage", io.getIntakeVoltage());
        SmartDashboard.putData("Arm/intakePID", intakePID);
        //Smart Dashboard Pivot
        SmartDashboard.putNumber("Arm/pivotDegree", io.getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/pivotVoltage", io.getPivotVoltage());
        SmartDashboard.putData("Arm/pivotPID", pivotPID);
    }

    public Command cPivotDegControl(){
        return runEnd(() -> {
            Rotation2d currentPos = io.getCurrentAngle();
            io.setArmVoltages(Optional.of(pivotFF.calculate(this.pivotSetpoint.getDegrees()) + pivotPID.calculate(currentPos.getDegrees(), this.pivotSetpoint.getDegrees())), Optional.empty());
        }, io::stop);
    }

    public Command cIntakeVelocityControl(double setpoint){
        return runEnd(() -> {
            double currentSpeed = io.getIntakeSpeed();

            io.setArmVoltages(Optional.empty(), Optional.of(intakeFF.calculate(setpoint) + intakePID.calculate(currentSpeed, setpoint)));
        }, io::stop);
    }

    /*
     * percent (the argument) should be a number between zero and one
     */
    public Command cIntakePercentDrive(double percent){
        return runEnd(() -> {
            io.setArmVoltages(Optional.empty(), Optional.of(percent*12));
        }, io::stop);
    }
}
