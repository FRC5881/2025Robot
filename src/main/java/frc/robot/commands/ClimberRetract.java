package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberRetract extends Command {
    private final ClimberSubsystem climber;

    public ClimberRetract(ClimberSubsystem climber) {
        this.climber = climber;
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        climber.retract();
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
