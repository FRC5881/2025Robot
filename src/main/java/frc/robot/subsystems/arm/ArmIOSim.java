package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

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

    private static SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 10, kArmInertia, armLength.in(Meters), -Math.PI/2, Math.PI/2, true, -Math.PI/2);
    private static FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), kWheelIntertia, 3), DCMotor.getNeo550(1));

    public ArmIOSim() {
    }

    @Override
    public void setArmVoltages(double pivotVoltage, double intakeVoltage) {
        armSim.setInputVoltage(pivotVoltage);
    }

    @Override
    public boolean hasAlgae() {
        return false;
    }

    @Override
    public Rotation2d currentAngle() {
        return new Rotation2d(armSim.getAngleRads());
    }

    @Override
    public double intakeSpeed() {
        return intakeSim.getAngularVelocityRPM();
    }

    public void periodic() {
        armSim.update(0.020);
        intakeSim.update(0.020);
    }
}