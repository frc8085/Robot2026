package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberConstants;

public class StartClimb extends Command{
    ClimberSubsystem climberSubsystem;

    public StartClimb(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize(){
        climberSubsystem.setClimbMotorVelocity(-ClimberConstants.kMaxClimbMotorVelocity);
    }
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimbMotor();
    }
    // i wasn't sure how I should
    @Override
    public boolean isFinished() {
        if (climberSubsystem.getClimbMotorPosition() <= ClimberConstants.kClimbMotorClimbPosition) {
            return true;
        } else {
            return false;
        }
    }
}
