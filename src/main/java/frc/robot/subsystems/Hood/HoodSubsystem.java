package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TalonFXSMotor;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFXSMotor hoodMotor;

    public HoodSubsystem() {
        hoodMotor = new TalonFXSMotor(HoodConstants.kHoodCanId);
    }

    public void go() {
        hoodMotor.setSpeed(0.1);
    }
}

