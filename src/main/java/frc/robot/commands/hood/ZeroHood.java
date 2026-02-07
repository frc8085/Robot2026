package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodSubsystem;

/**
 * Zeros the hood encoder so the hood's *current* position becomes 0 degrees.
 *
 * This is typically bound to a button (ex: driver A) so you can re-zero after enabling.
 *
 * This command finishes immediately after calling {@link HoodSubsystem#zeroEncoder()}.
 */
public class ZeroHood extends Command {
    private final HoodSubsystem hood;

    public ZeroHood(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        hood.zeroEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

