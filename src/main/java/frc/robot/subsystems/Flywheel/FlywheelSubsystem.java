package frc.robot.subsystems.Flywheel;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TalonFXMotor;

public class FlywheelSubsystem extends SubsystemBase {
    TalonFXMotor flywheelMain;
    TalonFXMotor flywheelFollow;

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
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kStatorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.kSupplyCurrentLimit;

        config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.kMotionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = FlywheelConstants.kMotionMagicJerk;
        
        this.flywheelMain = new TalonFXMotor(FlywheelConstants.kFlywheelMainCanId);
        flywheelMain.applyConfigs(config);
        flywheelMain.applySlotConfigs(slot0Configs);

        this.flywheelFollow = new TalonFXMotor(FlywheelConstants.kFlywheelFollowerCanID);

        flywheelFollow.applyConfigs(config);
        flywheelFollow.applySlotConfigs(slot0Configs);

        flywheelFollow.follow(flywheelMain, true);
    }
    
    private void setFlywheelRPS(double rps) {    
        flywheelMain.setMotorVelocity(rps);
    }

    private double MPStoRPS(double mps) {
        // mps Velocity 
        // rpm angular velocity
        // Velocity = Angular Velocity X Radius of the wheel
        // Angular Velocity = Velocity / Radius (m/s / m) = (1/s) (Radians per Second)
        // 1 Rotation = 2 * PI Radians
        // 1 Minute = 60 Seconds
        // Meters per second to rotations per minute
        // M/s -> R/M
        // M/s x 1/M = 1/s (Radians per second)
        // 1/s X 2PI/s = R/s (Rotations per second)
        double radPerSecond = mps / FlywheelConstants.kFlywheelRadius;
        return radPerSecond * (2 * Math.PI);
    }

    private double RPStoMPS(double rps) {
        double radPerSecond = rps / (2 * Math.PI);
        return radPerSecond * FlywheelConstants.kFlywheelRadius;
        
    }

    public void setFlywheelMetersPerSecond(double mps) {
        // is convert meters per second to rotations per minute
        System.out.println("shooter1");
        double rps = this.MPStoRPS(mps);
        this.setFlywheelRPS(rps);
    }

    public void stopFlywheel() {
        flywheelMain.setMotorVelocity(0);
    }

    public boolean isAtTargetMPS(double targetMPS) {
        // get the velocity (RPM) from the flywheel motor
        double currentRPS = flywheelMain.getVelocity();
        // convert the RPM -> to MPS
        double currentMPS = this.RPStoMPS(currentRPS);
        return Math.abs(currentMPS - targetMPS) <= FlywheelConstants.kFlywheelToleranceMPS;
    }

    public void go() {
        flywheelMain.setSpeed(.75);
        System.out.println("speed2");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}