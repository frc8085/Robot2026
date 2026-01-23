package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;

public class RunFlywheel extends Command {
    private final FlywheelSubsystem flywheelSubsystem;
    private final double targetRPM;

    public RunFlywheel(FlywheelSubsystem flywheelSubsystem, double targetRPM) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.targetRPM = targetRPM;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setFlywheelRPM(targetRPM);
    }

    @Override
    public boolean isFinished() {
        return flywheelSubsystem.isAtTargetRPM(targetRPM);
    }
}