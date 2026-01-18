package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TalonFXMotor;
import frc.robot.lib.PIDMotor;
import frc.robot.subsystems.Intake.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    TalonFXMotor intake;
    TalonFXMotor deploy;

    boolean isDeployed;

    public IntakeSubsystem()
    {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01;
        slot0Configs.kS = 0.20;

        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimit = 50;

        config.MotionMagic.MotionMagicAcceleration = 400;
        config.MotionMagic.MotionMagicJerk = 6000;
        
        this.intake = new TalonFXMotor(IntakeConstants.intakeMotorCanID);
        this.deploy = new TalonFXMotor(IntakeConstants.deployMotorCanID);
        

        intake.applyConfigs(config);

        intake.applySlotConfigs(slot0Configs);
        
    }

    public void runIntake(double speed)
    {
        if (isDeployed == true) {
            intake.setMotorVelocity(speed);
        }
    }
    
    public void deployIntake()
    {
        deploy.setMotorPosition(0);
        isDeployed = true;
    }

    public void retractIntake()
    {
        deploy.setMotorPosition(0);
        isDeployed = false;
    }

    public boolean isIntakeDeployed()
    {
        return isDeployed;
    }

}