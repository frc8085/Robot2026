package frc.robot.commands.drivetrain.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class RetractAndStopIntake extends Command {
    IntakeSubsystem intakeSubsystem;

    public RetractAndStopIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.stopIntake();
        intakeSubsystem.retractIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
