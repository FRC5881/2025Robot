package frc.robot.subsystems.armL1;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;
import frc.robot.subsystems.algaeArm.AlgaeArmSubsystem;

public class ArmL1Subsystem extends SubsystemBase {

    // TODO: Make proper new values
    private final SimpleMotorFeedforward feedforward_L1 = new SimpleMotorFeedforward(0,0,0);
    private final PIDController PID_L1 = new PIDController(0.15, 0.1, 0.05); //This is okay but needs more tuning

    public ArmL1IO io;

    //The angle system is the same as the AlgaeArm
    public Rotation2d L1Setpoint = new Rotation2d(Math.toRadians(-90));

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

    }

    public Command cDegControl(){
        return runEnd(() -> {
            Rotation2d currentPos = io.getCurrentAngle();
            io.setVoltage(feedforward_L1.calculate(this.L1Setpoint.getDegrees()) + PID_L1.calculate(currentPos.getDegrees(), this.L1Setpoint.getDegrees()));
        }, io::stop);
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
