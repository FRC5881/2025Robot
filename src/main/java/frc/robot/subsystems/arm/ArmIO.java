package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO {
    // output
    public void setArmVoltages(double pivotVoltage, double intakeVoltage);

    // inputs
    public boolean hasAlgae();

    public Rotation2d currentAngle();

    public double intakeSpeed(); 
    //I would like to add a unit to this but I couldn't figureout what class to use

    default void stop() {
        //Should this be in real? I don't think so as it's the same for both
        setArmVoltages(0, 0);
    }
}