package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class StopClimb extends Command{
    ClimberSubsystem climberSubsystem;

    public StopClimb(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize(){
        climberSubsystem.stopClimbMotor();
    }
}
