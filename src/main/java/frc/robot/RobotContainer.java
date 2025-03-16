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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.L2Subsystem;
import frc.robot.subsystems.algaeArm.AlgaeArmSubsystem;
import frc.robot.subsystems.armL1.ArmL1Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive;
  // private final Vision vision;
  private final L2Subsystem l2Subsystem = new L2Subsystem();
  private AlgaeArmSubsystem algae = new AlgaeArmSubsystem();
  private ClimberSubsystem climber = new ClimberSubsystem();
  private ArmL1Subsystem l1Subsystem = new ArmL1Subsystem();

  // Dashboard inputs
  private SwerveDriveSimulation driveSimulation = null;
  private final LoggedDashboardChooser<Command> autoChooser;

  private final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  // private final CommandXboxController m_copilotController = new
  // CommandXboxController(
  // OperatorConstants.kCopilotControllerPort);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});
        // this.vision =
        // new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOPhotonVision(camera0Name, robotToCamera0),
        // new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // Add the drive simulation to the arena
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        // vision =
        // new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new
        // VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption("Reverse", DriveCommands.driveReverse(drive));
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Reset gyro / odometry
    m_driverController
        .touchpad()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetOdometry(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // L2 Subsystem
    m_driverController.povLeft().whileTrue(l2Subsystem.outputLeftCommand());
    m_driverController.povRight().whileTrue(l2Subsystem.outputRightCommand());

    // Climber
    m_driverController.povUp().whileTrue(climber.cExtend());
    m_driverController.povDown().whileTrue(climber.cRetract());

    m_driverController.triangle().onTrue(l1Subsystem.cSetpoint(Constants.L1Constants.kHome));
    m_driverController.circle().onTrue(l1Subsystem.cSetpoint(Constants.L1Constants.kPrepare));
    m_driverController.square().onTrue(l1Subsystem.cSetpoint(Constants.L1Constants.kIntake));
    m_driverController.cross().onTrue(l1Subsystem.cSetpoint(Constants.L1Constants.kDrop));

    // Arm
    m_driverController
        .L2()
        .whileTrue(
            Commands.runEnd(() -> algae.intake(ArmConstants.kLowIntake), () -> algae.intake(0)));
    m_driverController
        .R2()
        .whileTrue(
            Commands.runEnd(() -> algae.intake(ArmConstants.kLowExpell), () -> algae.intake(0)));

    // m_driverController
    //     .L1()
    //     .whileTrue(
    //         Commands.runEnd(
    //             () -> algae.set(ArmConstants.kLow, ArmConstants.kLowIntake),
    //             () -> algae.intake(0)));
    // m_driverController
    //     .L2()
    //     .whileTrue(
    //         Commands.runEnd(
    //             () -> algae.set(ArmConstants.kHigh, ArmConstants.kHighIntake),
    //             () -> algae.intake(0)));
    // m_driverController
    //     .R1()
    //     .whileTrue(
    //         Commands.runEnd(
    //             () -> algae.set(ArmConstants.kLow, ArmConstants.kLowExpell),
    //             () -> algae.intake(0)));
    // m_driverController
    //     .R2()
    //     .whileTrue(
    //         Commands.runEnd(
    //             () -> algae.set(ArmConstants.kHigh, ArmConstants.kHighExpell),
    //             () -> algae.intake(0)));

    // m_driverController
    //     .L3()
    //     .whileTrue(
    //         Commands.runEnd(() -> algae.set(ArmConstants.kHomeDown, 0), () -> algae.intake(0)));
    // m_driverController
    //     .R3()
    //     .whileTrue(
    //         Commands.runEnd(() -> algae.set(ArmConstants.kHomeUp, 0), () -> algae.intake(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void zero() {
    l1Subsystem.zero();
    algae.zero();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    Logger.recordOutput(
        "FieldSimulation/RobotSimulation", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
