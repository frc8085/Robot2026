package frc.robot.commands.drivetrain.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class DeployAndRunIntake extends Command {
    IntakeSubsystem intakeSubsystem;

    public DeployAndRunIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.deployIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runIntake();
    }

    @Override
    public boolean isFinished() {
        if (intakeSubsystem.isMinimumRunPosition()) {
            return true;
        } else {
            return false;
        }
    }
    
   
    
}
