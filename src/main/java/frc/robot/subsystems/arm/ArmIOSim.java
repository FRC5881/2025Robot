package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    // Arm Simulation
    private static final Distance armLength = Inches.of(20);
    private static final Mass armMass = Pound.of(2);
    private static final Mass wheelMass = Pound.of(2);
    private static final Distance wheelRadius = Inches.of(3);
    private static final double wheelInertialConstant = 0.7; //A wheel loaded at the rim = 1, a cylinder=0.5, so I'm guessing

    private static final double kWheelIntertia = wheelInertialConstant * wheelMass.in(Kilograms) * wheelRadius.in(Meters);
    private static final double kArmInertia = (Math.pow(armLength.in(Meters), 2)) * ((armMass.in(Kilograms)/3) + wheelMass.in(Kilograms));

    SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 10, kArmInertia, armLength.in(Meters), -Math.PI/2, 3*Math.PI/4, true, -Math.PI/2);
    FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), kWheelIntertia, 3), DCMotor.getNeo550(1));

    public ArmIOSim() {
    }

    //outputs

    public double pivotVolts = 0;
    public double intakeVolts = 0;

    @Override
    public void setArmVoltages(Optional<Double> pivotVoltage, Optional<Double> intakeVoltage) {
        if (pivotVoltage.isPresent()){
            pivotSim.setInputVoltage(pivotVoltage.get());
            pivotVolts = pivotVoltage.get();
        }
        if (intakeVoltage.isPresent()){
            intakeSim.setInputVoltage(intakeVoltage.get());
            intakeVolts = intakeVoltage.get();
        }
    }
    
    @Override
    public void setArmVoltages(double pivotVoltage, double intakeVoltage) {
        pivotSim.setInputVoltage(pivotVoltage);
        intakeSim.setInputVoltage(intakeVoltage);
    }

    //inputs

    public double getPivotVoltage(){
        return pivotVolts;
    }

    public double getIntakeVoltage(){
        return intakeVolts;
    }

    @Override
    public boolean hasAlgae() {
        return false;
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return new Rotation2d(pivotSim.getAngleRads());
    }

    @Override
    public double getIntakeSpeed() {
        return intakeSim.getAngularVelocityRPM();
    }

}