package frc.robot.commands.turret;

import frc.robot.subsystems.turret.TurretSubsystem;

public class AimHood {
    TurretSubsystem turretSubsystem;
    int ref;

    public AimHood(TurretSubsystem ts, int ref){
        this.turretSubsystem = ts;
        this.ref = ref;
    }

    public void execute(){
        turretSubsystem.setTurretPosition(this.ref);
    }

    public boolean isFinished(){
        //isFinished always gets checked after execute so this will run once
        return true;
    }
}
