package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.lib.Motor;
import frc.robot.lib.PIDMotor;
import frc.robot.lib.TalonFXMotor;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.turret.TurretConstants.turretSlot0Configs;

public class TurretSubsystem {
    private PIDMotor turretMotor;
    private PIDMotor hoodMotor;
    private DigitalInput hoodLimitSwitch;
    private DriveSubsystem mDriveSubsystem = new DriveSubsystem();
    public TurretSubsystem(){
        hoodMotor = new TalonFXMotor(TurretConstants.hoodMotorCan);
        hoodLimitSwitch = new DigitalInput(TurretConstants.hoodLimitSwitchID);
        //configs turret motor
        var turretSlot0Configs = new Slot0Configs();
        //so I really didn't want to do it this way but I dont know any other ways so this is how it is :( (I tried to make it a function but that just added more overhead)
        turretSlot0Configs.kP = TurretConstants.turretSlot0Configs.kP;
        turretSlot0Configs.kI = TurretConstants.turretSlot0Configs.kI;
        turretSlot0Configs.kD = TurretConstants.turretSlot0Configs.kD;
        turretSlot0Configs.kV = TurretConstants.turretSlot0Configs.kV;
        turretSlot0Configs.kA = TurretConstants.turretSlot0Configs.kA;
        turretSlot0Configs.kS = TurretConstants.turretSlot0Configs.kS;
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.CurrentLimits.StatorCurrentLimit = 50;
        turretConfig.CurrentLimits.SupplyCurrentLimit = 50;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turretSlot0Configs.kMaxSpeed;
        turretConfig.MotionMagic.MotionMagicAcceleration = 200;
        turretConfig.MotionMagic.MotionMagicJerk = 6000;
        var turretTalon = new TalonFXMotor(TurretConstants.turretMotorCan);
        
            turretTalon.applyConfigs(turretConfig);

            turretTalon.applySlotConfigs(turretSlot0Configs);
        
        this.turretMotor = turretTalon;

        //configs hood motor
        var hoodSlot0Configs = new Slot0Configs();
        hoodSlot0Configs.kP = TurretConstants.hoodSlot0Configs.kP;
        hoodSlot0Configs.kI = TurretConstants.hoodSlot0Configs.kI;
        hoodSlot0Configs.kD = TurretConstants.hoodSlot0Configs.kD;
        hoodSlot0Configs.kV = TurretConstants.hoodSlot0Configs.kV;
        hoodSlot0Configs.kA = TurretConstants.hoodSlot0Configs.kA;
        hoodSlot0Configs.kS = TurretConstants.hoodSlot0Configs.kS;
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 50;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 50;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.turretSlot0Configs.kMaxSpeed;
        hoodConfig.MotionMagic.MotionMagicAcceleration = 200;
        hoodConfig.MotionMagic.MotionMagicJerk = 6000;
        var hoodTalon = new TalonFXMotor(TurretConstants.hoodMotorCan);
        
            hoodTalon.applyConfigs(hoodConfig);

            hoodTalon.applySlotConfigs(hoodSlot0Configs);
        
        this.hoodMotor = hoodTalon;
    }

    public boolean hoodLimitSwitchHit(){
        return hoodLimitSwitch.get();
    }

    public void setHoodPosition(double pos){
        pos = Math.min(Math.max(pos, TurretConstants.hoodMin), TurretConstants.hoodMax);
        hoodMotor.setMotorPosition(pos);
    }

    public void zeroHood(){
        //slow while zeroing
        hoodMotor.setMotorVelocity(-.15);;
    }
    public void setTurretPosition(double refrence){
        if (refrence > TurretConstants.turretMaxDeg){
            int offset = (int) ((refrence-TurretConstants.turretMaxDeg)/360) + 1;
            refrence -= offset*360;
        } else if (refrence < TurretConstants.turretMinDeg){
            int offset = (int) (Math.abs(refrence+TurretConstants.turretMinDeg)/360) + 1;
            refrence += offset*360;
        }
        turretMotor.setMotorPosition(refrence);
    }
}
