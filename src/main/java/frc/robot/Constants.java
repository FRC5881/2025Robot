// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class CANConstants {
    // Swerve Drive 1-8

    // Arm
    public static final int kPivotMotor = 10;
    public static final int kIntakeMotor = 11;

    // L1
    public static final int kMotorL1 = 20;

    // L2
    public static final int kL2Id = 26;

    // Climber
    public static final int kClimberId = 30;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }

  public static class ClimberConstants {
    public static final double kClimberSpeed = 0.5;
    public static final double kMinClimberSpeed = 0;
    public static final double kMaxTorque = 10;
  }

  public static class L2Constants {
    public static final double kL2CenterSpeed = 0.25;
    public static final double kL2ExitSpeed = 1;
  }

  public static class L1Constants {
    public static final Rotation2d kHome = Rotation2d.fromDegrees(0);
    public static final Rotation2d kPrepare = new Rotation2d(Math.toRadians(-70));
    public static final Rotation2d kDrop = Rotation2d.fromDegrees(-110);
    public static final Rotation2d kIntake = Rotation2d.fromDegrees(-40);
  }

  public static class PositionConstants {
    // Testing values
    public static final Rotation2d kAlgaeArmTestDown = new Rotation2d(-Math.PI / 2);
    public static final Rotation2d kAlgaeArmHorizontal = new Rotation2d(0.0);
    public static final Rotation2d kAlgaeArmUp = new Rotation2d(Math.PI / 2);

    // Algae Arm values
    public static final Rotation2d kAlgaeArmOut = new Rotation2d(Math.toRadians(-30));
    public static final Rotation2d kAlgaeArmDown = new Rotation2d(Math.toRadians(-60));
    public static final Rotation2d kAlgaeArmAway = new Rotation2d(Math.toRadians(80));

    // Algae Intake values
    public static final double kAlgaeIntakeIn = 3000;
    public static final double kAlgaeIntakeOut = -3000;
    public static final double kAlgaeIntakeHold = 0;
  }
}
