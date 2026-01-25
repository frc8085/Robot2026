package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;

public class RunFlywheel extends Command {
    private final FlywheelSubsystem flywheelSubsystem;
    private final double targetMPS;

    public RunFlywheel(FlywheelSubsystem flywheelSubsystem, double targetMPS) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.targetMPS = targetMPS;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setFlywheelMetersPerSecond(targetMPS);
    }

    @Override
    public boolean isFinished() {
        return flywheelSubsystem.isAtTargetMPS(targetMPS);
    }
}