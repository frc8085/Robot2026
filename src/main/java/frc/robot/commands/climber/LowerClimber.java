package frc.robot.commands.climber;

import java.lang.ref.Reference;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberConstants;
public class LowerClimber extends Command{
    ClimberSubsystem climberSubsystem;

    public LowerClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize(){
        climberSubsystem.setClimbMotorVelocity(ClimberConstants.MaxClimbMotorVelocity);
    }
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setClimbMotorVelocity(0);
    }
    @Override
    public boolean isFinished() {
        if ()
    }
}
