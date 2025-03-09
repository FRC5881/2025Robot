// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.OperatorConstants;
import frc.robot.subsystems.algaeArm.AlgaeArmSubsystem;
import frc.robot.subsystems.armL1.ArmL1Subsystem;

public class RobotContainer {
  
  private AlgaeArmSubsystem algaeArm = new AlgaeArmSubsystem();
  private ArmL1Subsystem coralArm = new ArmL1Subsystem();

  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            OperatorConstants.kDriverControllerPort);            
    // private final CommandPS5Controller m_copilotController = new CommandPS5Controller(
    //         OperatorConstants.kCopilotControllerPort);

  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {

    //TESTING CONTROLS:

    m_driverController.square().onTrue(Commands.runOnce(() -> {
      algaeArm.pivotSetpoint = Constants.PositionConstants.kAlgaeArmHorizontal;
    }));
    m_driverController.cross().onTrue(Commands.runOnce(() -> {
      algaeArm.pivotSetpoint = Constants.PositionConstants.kAlgaeArmTestDown;
    }));
    m_driverController.triangle().onTrue(Commands.runOnce(() -> {
      algaeArm.pivotSetpoint = Constants.PositionConstants.kAlgaeArmUp;
    }));
    m_driverController.circle().onTrue(Commands.runOnce(() -> {
      Rotation2d TriSetpoint = new Rotation2d(Math.toRadians(SmartDashboard.getNumber("Arm/mySetpoint", 20)));
      algaeArm.pivotSetpoint = TriSetpoint;
    }));

    //DRIVER CONTROLS:

    m_driverController.L1().onTrue(Commands.runOnce(() -> {
      algaeArm.cGrabAlgae();
    }));


    //Coral L1 Controls:

    // Will be used after testing
    // m_driverController.triangle().onTrue(Commands.runOnce(()->{
    //   coralArm.L1Setpoint = Constants.PositionConstants.kDropL1;
    // }));

    //COPILOT CONTROLS:

    //Coral L1 Controls:
    //TODO: change to copilot
    m_driverController.povUp().onTrue(Commands.runOnce(()->{
      coralArm.L1Setpoint = Constants.PositionConstants.kIntakeReadyL1;
    }));
    m_driverController.povDown().onTrue(Commands.runOnce(()->{
      coralArm.L1Setpoint = Constants.PositionConstants.kHomeL1;
    }));
    m_driverController.povRight().onTrue(Commands.runOnce(()->{
      coralArm.L1Setpoint = Constants.PositionConstants.kDropReadyL1;
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
