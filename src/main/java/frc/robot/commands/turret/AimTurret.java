package frc.robot.commands.turret;

import frc.robot.subsystems.turret.TurretSubsystem;
// I relised after making this it's useless but like it's here ig maybe we could use it as a base :)
public class AimTurret {
    TurretSubsystem turretSubsystem;

    public AimTurret(TurretSubsystem ts){
        this.turretSubsystem = ts;
    }

    public void execute(){
        turretSubsystem.setTurretPosition(0);
    }

    public boolean isFinished(){
        //isFinished always gets checked after execute so this will run once
        return true;
    }
}
