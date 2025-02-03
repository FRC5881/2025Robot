// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Utils.Constants.OperatorConstants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class RobotContainer {
  
  private ArmSubsystem arm = new ArmSubsystem();

  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);            
    private final CommandPS5Controller m_copilotController = new CommandPS5Controller(
            OperatorConstants.kCopilotControllerPort);

  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    Rotation2d horizontal = new Rotation2d(0.0);
    Rotation2d down = new Rotation2d(-Math.PI/2);
    Rotation2d up = new Rotation2d(Math.PI/2);

    m_driverController.square().onTrue(Commands.runOnce(() -> {
      arm.pivotSetpoint = horizontal;
    }));

    m_driverController.cross().onTrue(Commands.runOnce(() -> {
      arm.pivotSetpoint = down;
    }));

    m_driverController.triangle().onTrue(Commands.runOnce(() -> {
      arm.pivotSetpoint = up;
    }));


    //Testing
    m_driverController.circle().onTrue(Commands.runOnce(() -> {
      Rotation2d TriSetpoint = new Rotation2d(Math.toRadians(SmartDashboard.getNumber("Arm/mySetpoint", 20)));
      arm.pivotSetpoint = TriSetpoint;
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
