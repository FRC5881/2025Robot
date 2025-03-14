package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// + voltage = exits left
public class L2Subsystem extends SubsystemBase {
  public final SparkMax L2Motor;
  public final DigitalInput leftSensor = new DigitalInput(9);
  public final DigitalInput rightSensor = new DigitalInput(8);

  public L2Subsystem() {
    L2Motor = new SparkMax(Constants.CANConstants.kL2Id, MotorType.kBrushless);
    setDefaultCommand(centerCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    SmartDashboard.putBoolean("L1/Left Sensor", leftSensor.get());
    SmartDashboard.putBoolean("L1/Right Sensor", rightSensor.get());
  }

  public void output(double speed) {
    L2Motor.set(speed);
  }

  public void center(double speed) {
    if (leftSensor.get()) {
      L2Motor.set(-speed);
    } else if (rightSensor.get()) {
      L2Motor.set(+speed);
    } else L2Motor.set(0);
  }

  public void stop() {
    L2Motor.set(0);
  }

  public Command outputLeftCommand() {
    double speed = Constants.L2Constants.kL2ExitSpeed;
    return startEnd(() -> output(speed), () -> stop()).withName("output left");
  }

  public Command centerCommand() {
    double speed = Constants.L2Constants.kL2CenterSpeed;
    return startEnd(() -> center(speed), () -> stop()).withName("center");
  }

  public Command outputRightCommand() {
    double speed = -Constants.L2Constants.kL2ExitSpeed;
    return startEnd(() -> output(speed), () -> stop()).withName("output right");
  }
}
