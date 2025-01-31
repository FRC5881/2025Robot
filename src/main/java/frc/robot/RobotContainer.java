// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.ClimberSubsystem;

public class RobotContainer {
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Bind the PS5 controller buttons to the ClimberSubsystem commands
    ps5Controller.povUp().onTrue(climberSubsystem.cUpCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
