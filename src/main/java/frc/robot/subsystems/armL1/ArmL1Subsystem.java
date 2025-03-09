package frc.robot.subsystems.armL1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmL1Subsystem extends SubsystemBase {

    // TODO: Make proper new values
    private final SimpleMotorFeedforward feedforward_L1 = new SimpleMotorFeedforward(0,0,0);
    private final PIDController PID_L1 = new PIDController(0.024, 0.0035, 0.00015); //Tuned

    public ArmL1IO io;

    //The angle system is the same as the AlgaeArm
    public Rotation2d L1Setpoint = new Rotation2d(Math.toRadians(0));

    public ArmL1Subsystem() {
        if (RobotBase.isSimulation()) {
            io = new ArmL1IOSim();            
        }
        
        else {
            io = new ArmL1IOReal();            
        }
        this.setDefaultCommand(cDegControl());
    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber("L1/currentAngle", io.getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("L1/Setpoint", this.L1Setpoint.getDegrees());

        SmartDashboard.putNumber("L1/voltage", io.getVoltage());
        SmartDashboard.putData("L1/L1PID", PID_L1);

    }

    public Command cDegControl(){
        return runEnd(() -> {
            Rotation2d currentPos = io.getCurrentAngle();
            io.setVoltage(feedforward_L1.calculate(this.L1Setpoint.getDegrees()) + PID_L1.calculate(currentPos.getDegrees(), this.L1Setpoint.getDegrees()));
        }, io::stop).withName("cDegControl");
    }
    
    /*
     * Assumes that there is no coral in the L1 system
     */
    public Command cReadyL1(){
        return runEnd(() -> {
            //The coral station chute is at 55 degrees
            L1Setpoint = new Rotation2d(Math.toRadians(50));
            cDegControl();
            Commands.waitUntil(()->{return io.hasCoral();});
        }, ()->{
            Commands.waitSeconds(0.5);
            L1Setpoint = new Rotation2d(Math.toRadians(90));
            cDegControl();
            io.stop();
        });
    }

}
