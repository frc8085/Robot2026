package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.Hood.HoodConstants;

/**
 * Homes the hood by gently driving it into a mechanical hard stop, detected by a current spike.
 *
 * How it works:
 * 1. Apply a small constant voltage so the hood moves toward the hard stop.
 * 2. Watch motor current. When we hit the stop, the motor stalls and current rises.
 * 3. Once current is above the threshold for a short time, stop the motor and zero the encoder.
 *
 * This is typically bound to a button (ex: driver A) so you can home after enabling.
 *
 * IMPORTANT:
 * - This assumes you actually have a hard stop and that driving into it is safe.
 * - If it moves the wrong direction, flip {@link HoodConstants#kZeroingVoltageVolts}.
 */
public class ZeroHood extends Command {
    private final HoodSubsystem hood;
    private double aboveThresholdStartTimeSec = Double.NaN;

    public ZeroHood(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        aboveThresholdStartTimeSec = Double.NaN;
        hood.setHoodVoltage(HoodConstants.kZeroingVoltageVolts);
    }

    @Override
    public void execute() {
        // Keep applying voltage so the motor continues pushing toward the stop.
        hood.setHoodVoltage(HoodConstants.kZeroingVoltageVolts);

        double amps = hood.getStatorCurrentAmps();
        double nowSec = Timer.getFPGATimestamp();

        if (amps >= HoodConstants.kZeroingStatorCurrentThresholdAmps) {
            if (Double.isNaN(aboveThresholdStartTimeSec)) {
                aboveThresholdStartTimeSec = nowSec;
            }
        } else {
            // Current dropped back down, so reset the timer.
            aboveThresholdStartTimeSec = Double.NaN;
        }
    }

    @Override
    public boolean isFinished() {
        if (Double.isNaN(aboveThresholdStartTimeSec)) {
            return false;
        }
        double nowSec = Timer.getFPGATimestamp();
        return (nowSec - aboveThresholdStartTimeSec) >= HoodConstants.kZeroingCurrentTimeSec;
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop output when the command ends.
        hood.setHoodVoltage(0.0);

        // Only declare "home found" and zero when we finished normally.
        if (!interrupted) {
            hood.zeroEncoder();
        }
    }
}
