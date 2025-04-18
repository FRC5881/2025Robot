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
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
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

    // L1
    public static final int kL1Motor = 20;

    // L2
    public static final int kL2Id = 26;
    public static final int kL2RampId = 27;

    // Climber
    public static final int kClimberId = 30;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
  }

  public static class ClimberConstants {}

  public static class L1Constants {
    public static final Rotation2d kHome = Rotation2d.fromDegrees(0);
    public static final Rotation2d kIntake = Rotation2d.fromDegrees(35);
    public static final Rotation2d kPrepare = new Rotation2d(Math.toRadians(80));
    public static final Rotation2d kDrop = Rotation2d.fromDegrees(110);

    public static final double kL1Ratio = 4 * 5;
  }

  public static class L2Constants {
    public static final double kL2CenterSpeed = 0.25 * 4.0 / 5.0;
    public static final double kL2ExitSpeed = 0.75;
    public static final double kL2RampRatio = 5 * 5;
    public static final double kL2RampPosition = 0.8;
  }
}
