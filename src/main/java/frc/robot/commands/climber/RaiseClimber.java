package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberConstants;

public class RaiseClimber extends Command{
    ClimberSubsystem climberSubsystem;

    public RaiseClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize(){
        climberSubsystem.setClimbMotorVelocity(ClimberConstants.kMaxClimbMotorVelocity);
    }
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimbMotor();
    }
    @Override
    public boolean isFinished() {
        if (climberSubsystem.getClimbMotorPosition() >= ClimberConstants.kMaxClimbMotorPosition) {
            return true;
        } else {
            return false;
        }
    }
}
