package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;

public class RunFlywheel extends Command {
    private final FlywheelSubsystem flywheelSubsystem;
    private final double targetRPS;

    public RunFlywheel(FlywheelSubsystem flywheelSubsystem, double targetRPS) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.targetRPS = targetRPS;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setFlywheelRPS(targetRPS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}