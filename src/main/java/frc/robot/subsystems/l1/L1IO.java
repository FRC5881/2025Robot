package frc.robot.subsystems.l1;

import edu.wpi.first.math.geometry.Rotation2d;

public interface L1IO {
  public void setVoltage(double voltage);

  public void zero();

  public double getTemp();

  public double getCurrent();

  public boolean getLimit();

  public double getVoltage();

  public Rotation2d getCurrentAngle();

  default void stop() {
    setVoltage(0);
  }
}
