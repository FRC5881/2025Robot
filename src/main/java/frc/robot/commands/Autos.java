package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.L1Constants;
import frc.robot.subsystems.L2Subsystem;
import frc.robot.subsystems.armL1.ArmL1Subsystem;
import frc.robot.subsystems.drive.Drive;

public class Autos {
  public static void setup(ArmL1Subsystem l1, L2Subsystem l2, Drive drive) {
    NamedCommands.registerCommand("L1-Ready", prepL1(l1));
    NamedCommands.registerCommand("L1", dropL1(l1));
    NamedCommands.registerCommand("L2", autoL2(l2, drive));
    NamedCommands.registerCommand("180 zero", Commands.runOnce(
        () -> {
          drive.resetOdometry(
              new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg));
        }));
  }

  public static Command dropL1(ArmL1Subsystem l1) {
    return l1.cSetpoint(L1Constants.kDrop);
  }

  public static Command prepL1(ArmL1Subsystem l1) {
    return l1.cSetpoint(L1Constants.kPrepare);
  }

  public static Command autoL2(L2Subsystem l2, Drive drive) {
    return Commands.waitUntil(() -> l2.reefSensor.get())
        .andThen(
            Commands.run(drive::stopWithX, drive).alongWith(l2.sendLeftCommand().withTimeout(2)));
  }
}
