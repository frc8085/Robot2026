package frc.robot.lib;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class TalonFXMotor implements PIDMotor {

    private final TalonFX TalonFX;

    private MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0);
    private MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0);
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<AngularVelocity> motorVelocity;

    public TalonFXMotor(int id) {
        TalonFX = new TalonFX(id, CANBus.roboRIO());
        motorPosition = TalonFX.getPosition();
        motorVelocity = TalonFX.getVelocity();
    }

    public void applySlotConfigs(Slot0Configs config) {
        TalonFX.getConfigurator().apply(config);
    }

    public void applyConfigs(TalonFXConfiguration config) {
        TalonFX.getConfigurator().apply(config);
    }

    @Override
    public void setMotorPosition(double reference) {
        motionMagicPositionControl.Position = reference;
        TalonFX.setControl(motionMagicPositionControl);
    }

    @Override
    public void setMotorVelocity(double reference) {
        motionMagicVelocityControl.Velocity = reference;
        TalonFX.setControl(motionMagicVelocityControl);
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
        TalonFX.setPosition(reference);
    }

    @Override
    public void setSpeed(double speed) {
        TalonFX.set(speed);
    }

    @Override
    public double getSpeed() {
        return TalonFX.get();
    }

    public int getID() {
        return TalonFX.getDeviceID();
    }

    public void follow(TalonFXMotor leader, boolean inverted) {
        var aligned = MotorAlignmentValue.Aligned;
        if (inverted) {
            aligned = MotorAlignmentValue.Opposed;
        }
        TalonFX.setControl(new Follower(leader.getID(), aligned));
    }
}
