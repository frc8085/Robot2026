package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodSubsystem;

/**
 * Command that tells the hood to move to a specific angle and finishes when it arrives.
 *
 * This is intentionally written in a "simple" style for readability:
 * - initialize(): send the target to the subsystem
 * - isFinished(): ask the subsystem if it is at the target
 */
public class SetHoodAngle extends Command {
    private final HoodSubsystem hood;
    private final double targetDegrees;

    public SetHoodAngle(HoodSubsystem hood, double targetDegrees) {
        this.hood = hood;
        this.targetDegrees = targetDegrees;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.setHoodAngleDegrees(targetDegrees);
    }

    @Override
    public boolean isFinished() {
        return hood.atTargetPosition();
    }
}

