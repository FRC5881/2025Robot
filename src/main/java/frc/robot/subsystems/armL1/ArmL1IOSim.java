package frc.robot.subsystems.armL1;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmL1IOSim implements ArmL1IO {

    //TODO: Change values
    private static final Distance armLength = Inches.of(20);
    private static final Mass armMass = Pound.of(2);
    private static final double kArmInertia = (Math.pow(armLength.in(Meters), 2)) * ((armMass.in(Kilograms)/3));
    
    SingleJointedArmSim armL1Sim = new SingleJointedArmSim(DCMotor.getNEO(1), 10, kArmInertia, armLength.in(Meters), -Math.PI/2, 3*Math.PI/4, true, -Math.PI/2);


    public ArmL1IOSim() {
    }

    public double voltage = 0;
    public boolean coralL1 = false;

    @Override
    public void setVoltage(double voltage) {
        armL1Sim.setInputVoltage(voltage);
    }

    @Override
    public double getVoltage() {
        return voltage;
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return new Rotation2d(armL1Sim.getAngleRads());
    }

    @Override
    public boolean hasCoral() {
        return coralL1;
    }

}
