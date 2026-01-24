package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberConstants;
public class DeployClimber extends Command{
    ClimberSubsystem climberSubsystem;

    public DeployClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize(){
        climberSubsystem.setDeployPosition(ClimberConstants.kL1ClimbMaxRot);
    }
    @Override
    public void end(boolean interrupted) {
        new RaiseClimber(climberSubsystem).schedule();
    }

    @Override
    public boolean isFinished() {
        // if the deploy motor is past a certain position, it will allow the climber motor to raise the climb hook
        if (climberSubsystem.getDeployPosition() >= ClimberConstants.kL1ClimbMinRaiseRot) {
            return true;
        } else {
            return false;
        }
    }
}
