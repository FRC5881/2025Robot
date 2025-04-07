package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.L1Constants;
import frc.robot.subsystems.L2Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.l1.L1Subsystem;

public class Autos {
  public static void setup(L1Subsystem l1, L2Subsystem l2, Drive drive) {
    NamedCommands.registerCommand("L1-Ready", prepL1(l1));
    NamedCommands.registerCommand("L1", dropL1(l1));
    NamedCommands.registerCommand("L2", autoL2(l2, drive));
  }

  public static Command dropL1(L1Subsystem l1) {
    return l1.cSetpoint(L1Constants.kDrop).andThen(Commands.waitSeconds(1));
  }

  public static Command prepL1(L1Subsystem l1) {
    return l1.cSetpoint(L1Constants.kPrepare);
  }

  public static Command autoL2(L2Subsystem l2, Drive drive) {
    return Commands.waitUntil(() -> l2.reefSensor.get())
        .andThen(
            Commands.run(drive::stopWithX, drive).alongWith(l2.sendLeftCommand().withTimeout(2)));
  }
}
