package frc.robot.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Minimal wrapper around CTRE Phoenix 6 {@link TalonFXS} that matches the
 * {@link PIDMotor} API used by this codebase.
 */
public class TalonFXSMotor implements PIDMotor {

    private final TalonFXS talonFXS;

    private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;

    public TalonFXSMotor(int id) {
        talonFXS = new TalonFXS(id, CANBus.roboRIO());
        motorPosition = talonFXS.getPosition();
        motorVelocity = talonFXS.getVelocity();

        // Hard-coded for the 2026 season: we only expect to use TalonFXS with the Minion motor (JST).
        talonFXS.getConfigurator().apply(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST));
    }

    public void applySlotConfigs(Slot0Configs config) {
        talonFXS.getConfigurator().apply(config);
    }

    public void applyConfigs(TalonFXSConfiguration config) {
        // Hard-coded for the 2026 season: we only expect to use TalonFXS with the Minion motor (JST).
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        talonFXS.getConfigurator().apply(config);
    }

    @Override
    public void setMotorPosition(double reference) {
        motionMagicPositionControl.Position = reference;
        talonFXS.setControl(motionMagicPositionControl);
    }

    @Override
    public void setMotorVelocity(double reference) {
        motionMagicVelocityControl.Velocity = reference;
        talonFXS.setControl(motionMagicVelocityControl);
    }

    @Override
    public double getPosition() {
        motorPosition.refresh();
        return motorPosition.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        motorVelocity.refresh();
        return motorVelocity.getValueAsDouble();
    }

    @Override
    public void setEncoderPosition(double reference) {
        talonFXS.setPosition(reference);
    }

    @Override
    public void setSpeed(double speed) {
        talonFXS.set(speed);
    }

    @Override
    public double getSpeed() {
        return talonFXS.get();
    }

    public int getID() {
        return talonFXS.getDeviceID();
    }

    public void follow(TalonFXSMotor leader, boolean inverted) {
        var aligned = MotorAlignmentValue.Aligned;
        if (inverted) {
            aligned = MotorAlignmentValue.Opposed;
        }
        talonFXS.setControl(new Follower(leader.getID(), aligned));
    }
}
