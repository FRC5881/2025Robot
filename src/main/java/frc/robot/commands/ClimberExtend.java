package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtend extends Command {
    private final ClimberSubsystem climber;

    public ClimberExtend(ClimberSubsystem climber) {
        this.climber = climber;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climber.extend();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
