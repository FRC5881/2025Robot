package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

// + voltage = exits left
public class L2Subsystem extends SubsystemBase {
  public final SparkMax conveyor = new SparkMax(Constants.CANConstants.kL2Id, MotorType.kBrushless);
  public final SparkMax ramp = new SparkMax(Constants.CANConstants.kL2RampId, MotorType.kBrushless);

  public final DigitalInput reefSensor = new DigitalInput(8);
  public final DigitalInput coralSensor = new DigitalInput(9);

  public L2Subsystem() {
    var conveyorConfig = new SparkMaxConfig();
    conveyorConfig.smartCurrentLimit(20);
    conveyor.configure(
        conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var rampConfig = new SparkMaxConfig();
    rampConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
    rampConfig.encoder.positionConversionFactor(1.0 / (5 * 5));
    rampConfig.closedLoop.outputRange(-6, 6).p(1.0);
    ramp.configure(rampConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(centerCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("L2/Left Sensor", coralSensor.get());
    SmartDashboard.putBoolean("L2/Reef Sensor", reefSensor.get());
    SmartDashboard.putNumber("L2/Ramp Current", ramp.getOutputCurrent());
    SmartDashboard.putNumber("L2/Ramp Voltage", ramp.getAppliedOutput() * ramp.getBusVoltage());
    SmartDashboard.putNumber("L2/Ramp Position", ramp.getEncoder().getPosition());
    SmartDashboard.putNumber("L2/Ramp Temperature", ramp.getMotorTemperature());
  }

  public void startRamp() {}

  public void stopRamp() {
    ramp.stopMotor();
  }

  private void set(double speed) {
    conveyor.set(speed);
  }

  public static double setpoint = 0.7;

  private void center(double speed) {
    ramp.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    if (coralSensor.get()) {
      conveyor.set(speed);
    } else {
      conveyor.stopMotor();
    }
  }

  private void stop() {
    conveyor.stopMotor();
  }

  public Command sendLeftCommand() {
    double speed = -Constants.L2Constants.kL2ExitSpeed;
    return runEnd(() -> set(speed), () -> stop()).withName("output left");
  }

  public Command centerCommand() {
    double speed = Constants.L2Constants.kL2CenterSpeed;
    return runEnd(() -> center(speed), () -> stop()).withName("center");
  }

  public Command sendRightCommand() {
    double speed = Constants.L2Constants.kL2ExitSpeed;
    return runEnd(() -> set(speed), () -> stop()).withName("output right");
  }

  public Trigger reefTrigger() {
    return new Trigger(reefSensor::get);
  }

  public void zero() {
    ramp.getEncoder().setPosition(0);
  }
}
