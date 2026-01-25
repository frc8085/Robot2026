package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;

public class StopFlywheel extends Command {
    private final FlywheelSubsystem flywheelSubsystem;

    public StopFlywheel(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
