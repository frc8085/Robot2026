package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.SlotConfigs;

public class TurretConstants {
    public static final int turretMotorCan = 0;
    public static final int hoodMotorCan = 0;
    public static final int hoodLimitSwitchID = 0;
    public class turretSlot0Configs {
        public static final double kP = 0.12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kS = 0.20;
        public static final double kMaxSpeed = 80;
    }
    public class hoodSlot0Configs {
        public static final double kP = 0.12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kS = 0.20;
        public static final double kMaxSpeed = 80;
    }
    public static final double hoodMax = 0;
    public static final double hoodMin = 0;
    public static final double turretMaxDeg = 270;
    public static final double turretMinDeg = -270;
}