package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TalonFXSMotor;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFXSMotor hoodMotor;

    // The most-recent commanded hood target. Used by atTargetPosition().
    private double targetHoodAngleDegrees = 0.0;

    public HoodSubsystem() {
        hoodMotor = new TalonFXSMotor(HoodConstants.kHoodCanId);

        /*
         * Configure the motor for position control.
         *
         * We keep this configuration intentionally minimal so a new programmer can follow it.
         * Add current limits / soft limits / etc. as needed later.
         */
        var config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // hood should hold position when not commanded

        /*
         * Current limits (protects motor/wiring).
         *
         * - Stator current: current through the motor itself (torque-producing).
         * - Supply current: current drawn from the battery through the controller.
         *
         */
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.kStatorCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.kSupplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLowerLimit = HoodConstants.kSupplyCurrentLowerLimitAmps;
        config.CurrentLimits.SupplyCurrentLowerTime = HoodConstants.kSupplyCurrentLowerTimeSec;

        // Best practice: apply Slot0 + MotionMagic using one configuration object, then apply once.
        config.Slot0.kP = HoodConstants.kPositionP;
        config.Slot0.kI = HoodConstants.kPositionI;
        config.Slot0.kD = HoodConstants.kPositionD;

        config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kMotionMagicCruiseVelocityRps;
        config.MotionMagic.MotionMagicAcceleration = HoodConstants.kMotionMagicAccelerationRps2;
        config.MotionMagic.MotionMagicJerk = HoodConstants.kMotionMagicJerkRps3;

        hoodMotor.applyConfigs(config);
    }

    /**
     * Simple open-loop output. Kept from the first bring-up so we can still "bump"
     * the motor for quick testing.
     */
    public void go() {
        hoodMotor.setSpeed(0.1);
    }

    /**
     * Open-loop voltage control. The motor will spin continuously at the requested voltage
     * until another command changes the output (position control, speed, stop, etc.).
     *
     * @param volts Motor voltage to apply (typically between -12 and +12).
     */
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    /**
     * Stator current is the motor phase current. It spikes strongly when the mechanism stalls
     * (for example, when you drive into a hard stop).
     */
    public double getStatorCurrentAmps() {
        return hoodMotor.getStatorCurrentAmps();
    }

    /**
     * Command the hood to an absolute angle (degrees).
     *
     * "Absolute" here means relative to whatever the encoder currently considers 0.
     * Use {@link #zeroEncoder()} to define where 0 degrees is.
     */
    public void setHoodAngleDegrees(double hoodAngleDegrees) {
        targetHoodAngleDegrees = hoodAngleDegrees;

        // CTRE position units for TalonFXS are rotations of the motor sensor.
        double motorRotations = hoodDegreesToMotorRotations(hoodAngleDegrees);
        hoodMotor.setMotorPosition(motorRotations);
    }

    /**
     * Makes the hood's *current* position become 0 degrees by zeroing the motor's encoder.
     *
     * This is commonly bound to a button so drivers can re-zero after enabling.
     */
    public void zeroEncoder() {
        hoodMotor.setEncoderPosition(0.0);
        targetHoodAngleDegrees = 0.0;
    }

    /**
     * Returns the current hood angle in degrees (based on the motor encoder + our ratio math).
     */
    public double getHoodAngleDegrees() {
        return motorRotationsToHoodDegrees(hoodMotor.getPosition());
    }

    /**
     * True when the hood is close enough to the last commanded position.
     *
     * Used by {@code SetHoodAngle} command to decide when to finish.
     */
    public boolean atTargetPosition() {
        double errorDeg = targetHoodAngleDegrees - getHoodAngleDegrees();
        return Math.abs(errorDeg) <= HoodConstants.kPositionToleranceDegrees;
    }

    /*
     * --------------------
     * Angle conversion math
     * --------------------
     *
     * We have a chain of reductions between the motor and the hood.
     * This helper code converts between:
     * - hood angle (degrees)
     * - motor sensor position (rotations)
     *
     * Dagram (driver -> driven, where speed ratio = driverTeeth / drivenTeeth):
     *
     *   Motor
     *     |
     *     v
     *   [Planetary 20:1]
     *     |
     *     v
     *   Pulley1 (T1)  ==belt==>  Pulley2 (T2) --same shaft--> Pulley3 (T3) ==belt==> Pulley4 (T4) -> Hood
     *
     * Belt math:
     * - Pulley2 rotations = Pulley1 rotations * (T1 / T2)
     * - Pulley4 rotations = Pulley3 rotations * (T3 / T4)
     *
     * Therefore hood rotations (same as Pulley4 rotations):
     *   hoodRot = (motorRot / planetary) * (T1/T2) * (T3/T4)
     *
     * Solve for motor rotations:
     *   motorRot = hoodRot * planetary * (T2/T1) * (T4/T3)
     */
    private static double motorRotationsPerHoodRotation() {
        double planetary = HoodConstants.kPlanetaryReduction;

        double t1 = HoodConstants.kPulley1Teeth;
        double t2 = HoodConstants.kPulley2Teeth;
        double t3 = HoodConstants.kPulley3Teeth;
        double t4 = HoodConstants.kPulley4Teeth;

        return planetary * (t2 / t1) * (t4 / t3);
    }

    private static double hoodDegreesToMotorRotations(double hoodDegrees) {
        double hoodRotations = hoodDegrees / 360.0;
        return hoodRotations * motorRotationsPerHoodRotation();
    }

    private static double motorRotationsToHoodDegrees(double motorRotations) {
        double hoodRotations = motorRotations / motorRotationsPerHoodRotation();
        return hoodRotations * 360.0;
    }
}
