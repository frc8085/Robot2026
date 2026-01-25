package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ClimberConstants;
public class RetractClimber extends Command{
    ClimberSubsystem climberSubsystem;

    public RetractClimber(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }
    @Override
    public void initialize(){
        new LowerClimber(climberSubsystem).schedule();
    }
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setDeployPosition(ClimberConstants.kL1ClimbMaxRot);
    }


}
