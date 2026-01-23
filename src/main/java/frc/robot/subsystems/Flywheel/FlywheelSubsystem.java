package frc.robot.subsystems.Flywheel;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TalonFXMotor;

public class FlywheelSubsystem extends SubsystemBase {
    TalonFXMotor flywheel;

    public FlywheelSubsystem() {
       var slot0Configs = new Slot0Configs();
        slot0Configs.kP = FlywheelConstants.kP;
        slot0Configs.kI = FlywheelConstants.kI;
        slot0Configs.kD = FlywheelConstants.kD;
        slot0Configs.kV = FlywheelConstants.kV;
        slot0Configs.kA = FlywheelConstants.kA;
        slot0Configs.kS = FlywheelConstants.kS;

        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Flywheels usually coast when not powered
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimit = 50;

        config.MotionMagic.MotionMagicAcceleration = 400;
        config.MotionMagic.MotionMagicJerk = 6000;
        
        this.flywheel = new TalonFXMotor(FlywheelConstants.kFlywheelMotorCanId);
        

        flywheel.applyConfigs(config);

        flywheel.applySlotConfigs(slot0Configs);
    }

    public void setFlywheelRPM(double rpm) {    
        flywheel.setMotorVelocity(rpm);
    }

    public void stopFlywheel() {
        flywheel.setMotorVelocity(0);
    }

    public boolean isAtTargetRPM(double targetRPM) {
        double currentRPM = flywheel.getVelocity();
        return Math.abs(currentRPM - targetRPM) <= FlywheelConstants.kFlywheelToleranceRPM;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}