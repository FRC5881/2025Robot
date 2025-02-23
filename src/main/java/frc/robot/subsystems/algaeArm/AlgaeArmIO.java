package frc.robot.subsystems.algaeArm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AlgaeArmIO {
    // output
    public void setArmVoltages(Optional<Double> pivotVoltage, Optional<Double> intakeVoltage);
    public void setArmVoltages(double pivotVoltage, double intakeVoltage);

    // inputs

    /*
     * Returns [intakeVolts, pivotVolts]
     */
    public double getPivotVoltage();
    public double getIntakeVoltage();
    public boolean hasAlgae();

    public Rotation2d getCurrentAngle();

    public double getIntakeSpeed(); 
    //I would like to add a unit to this but I couldn't figureout what class to use

    default void stop() {
        //Should this be in real? I don't think so as it's the same for both
        setArmVoltages(0,0);
    }
}