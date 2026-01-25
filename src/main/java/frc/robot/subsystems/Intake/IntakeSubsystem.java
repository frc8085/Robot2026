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

    public IntakeSubsystem()
    {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kS = IntakeConstants.kS;

        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstant.StatorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstant.SupplyCurrentLimit;

        config.MotionMagic.MotionMagicAcceleration = IntakeConstant.MMAcceleration;
        config.MotionMagic.MotionMagicJerk = IntakeConstants.MMJerk;
        
        this.intake = new TalonFXMotor(IntakeConstants.intakeMotorCanID);
        this.deploy = new TalonFXMotor(IntakeConstants.deployMotorCanID);
        

        intake.applyConfigs(config);

        intake.applySlotConfigs(slot0Configs);
        
    }

    public void runIntake()
    {
        intake.setMotorVelocity(IntakeConstants.motorSpeed);
    }

    public void stopIntake()
    {
        intake.setMotorVelocity(0);
    }
    
    public void deployIntake()
    {
        deploy.setMotorPosition(IntakeConstants.intakeDeployedPosition);
    }

    public void retractIntake()
    {
        deploy.setMotorPosition(IntakeConstants.intakeRetractedPosition);
    }

    public boolean isIntakeDeployed()
    {
        return deploy.getPosition() >= IntakeConstants.intakeDeployedPosition;
    }

    public boolean isMinimumRunPosition()
    {
        return deploy.getPosition() >= IntakeConstants.intakeMinimumRunPosition;
    }

}