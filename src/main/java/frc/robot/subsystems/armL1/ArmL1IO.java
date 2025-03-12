package frc.robot.subsystems.armL1;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmL1IO {
  public void setVoltage(double voltage);

  public double getVoltage();

  public Rotation2d getCurrentAngle();

  default void stop() {
    setVoltage(0);
  }
}
