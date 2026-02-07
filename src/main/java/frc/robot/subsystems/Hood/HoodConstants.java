package frc.robot.subsystems.Hood;

public final class HoodConstants {
    private HoodConstants() {
    }

    public static final int kHoodCanId = 23;

    /*
     * Current limits (amps).
     *
     * These protect the motor, wiring, and breaker by limiting current draw.
     * Values here are conservative "starting points" and should be tuned on the real robot.
     */
    public static final double kStatorCurrentLimitAmps = 15.0;
    public static final double kSupplyCurrentLimitAmps = 10.0;
    public static final double kSupplyCurrentLowerLimitAmps = 10.0;
    public static final double kSupplyCurrentLowerTimeSec = 1.0;

    /*
     * Homing / zeroing by current spike (driving into a hard stop).
     *
     * We apply a small constant voltage until the motor "stalls" against the stop.
     * Stall = current above a threshold for a short amount of time.
     *
     * IMPORTANT: pick a voltage that is strong enough to reach the stop, but not so
     * strong that it slams the mechanism.
     *
     * NOTE: the sign of this voltage depends on your mechanism. If pressing the zero button
     * moves the hood the wrong direction, flip the sign.
     */
    public static final double kZeroingVoltageVolts = -1.0;
    public static final double kZeroingStatorCurrentThresholdAmps = 10.0;
    public static final double kZeroingCurrentTimeSec = 0.15;

    /*
     * Position control tuning (CTRE Phoenix 6, Slot0).
     *
     * When we command a position, the motor controller computes:
     *   output ~= kP * positionError + kI * integral(error) + kD * derivative(error)
     *
     * For Motion Magic position control, position is in *motor rotations*.
     * That means:
     *   kP units are roughly "volts per rotation of error" when using MotionMagicVoltage.
     *
     * These values are intentionally simple to start with.
     * - Start with P-only.
     * - Increase kP until it moves crisply but does not oscillate.
     */
    public static final double kPositionP = 5.0;
    public static final double kPositionI = 0.0;
    public static final double kPositionD = 0.0;

    // Our command ends when we are within this tolerance of the target.
    // Tune this on the real robot (too tight can cause commands to "never finish").
    public static final double kPositionToleranceDegrees = 1.0;

    /*
     * Motion Magic profile limits.
     *
     * CTRE units (Phoenix 6):
     * - Cruise velocity: motor rotations per second (rps)
     * - Acceleration: motor rotations per second^2 (rps^2)
     * - Jerk: motor rotations per second^3 (rps^3)
     *
     * These are "reasonable starting points" for a hood. Expect to tune them.
     */
    public static final double kMotionMagicCruiseVelocityRps = 6.0;
    public static final double kMotionMagicAccelerationRps2 = 30.0;
    public static final double kMotionMagicJerkRps3 = 300.0;

    /*
     * Mechanical ratios (motor -> hood).
     *
     * We convert "hood degrees" into "motor rotations" by multiplying through the whole ratio chain.
     *
     * Chain (as built on the robot):
     *
     *   [Minion Motor] -> [Planetary 20:1] -> Pulley #1 (23T)
     *                                      belt
     *                                      v
     *                                  Pulley #2 (23T) --same shaft--> Pulley #3 (18T)
     *                                                                   belt
     *                                                                   v
     *                                                               Pulley #4 (36T) -> Hood (1:1)
     */
    public static final double kPlanetaryReduction = 20.0; // 20 motor turns = 1 gearbox output turn

    public static final int kPulley1Teeth = 23;
    public static final int kPulley2Teeth = 23;
    public static final int kPulley3Teeth = 18;
    public static final int kPulley4Teeth = 36;

    static {
        /*
         * Fail fast on impossible configurations.
         *
         * If the stator current limit is set below the homing current threshold,
         * the motor controller will clamp current and we will never detect the "stall spike".
         */
        if (kZeroingStatorCurrentThresholdAmps >= kStatorCurrentLimitAmps) {
            throw new IllegalStateException(
                    "HoodConstants misconfigured: kZeroingStatorCurrentThresholdAmps ("
                            + kZeroingStatorCurrentThresholdAmps
                            + ") must be less than kStatorCurrentLimitAmps ("
                            + kStatorCurrentLimitAmps
                            + ").");
        }
    }
}
