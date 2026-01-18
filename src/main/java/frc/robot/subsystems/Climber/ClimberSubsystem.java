package frc.robot.subsystems.Climber;

import frc.robot.lib.PIDMotor;
import frc.robot.lib.TalonFXMotor;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {
    
    private PIDMotor l1Climber;


    public ClimberSubsystem() {
        // CanID

        var slot0Configs = new Slot0Configs();
 

        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLimit = 50;

        config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.Slot0Configs.MAXSPEED;
        config.MotionMagic.MotionMagicAcceleration = 200;
        config.MotionMagic.MotionMagicJerk = 6000;

        var talon = new TalonFXMotor(ClimberConstants.L1CLIMBMOTORID);

            talon.applyConfigs(config);

            talon.applySlotConfigs(slot0Configs);
        
        this.l1Climber = talon;

    }


    public void setClimbPosition(double reference) {
        if (reference > ClimberConstants.L1CLIMBMAXROT) {
            reference = ClimberConstants.L1CLIMBMAXROT;
        } else if (reference < ClimberConstants.L1CLIMBMINROT) {
            reference = ClimberConstants.L1CLIMBMINROT;
        }
        this.l1Climber.setMotorPosition(reference);
    }

}
