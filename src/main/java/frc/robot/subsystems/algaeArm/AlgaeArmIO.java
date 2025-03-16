package frc.robot.subsystems.algaeArm;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AlgaeArmIO {
  // output
  public void setArmVoltages(double pivotVoltage, double intakeVoltage);

  public void zero();

  // inputs
  public double getPivotVoltage();

  public double getIntakeVoltage();

  public Rotation2d getCurrentAngle();

  public double getIntakeSpeed();

  default void stop() {
    setArmVoltages(0, 0);
  }
}
