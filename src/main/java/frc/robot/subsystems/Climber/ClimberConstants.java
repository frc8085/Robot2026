package frc.robot.subsystems.Climber;

public class ClimberConstants {
    

    public static final int CanL1Climb = 21;
public static final int CanL2Climb = 22;
    public static final double kL1ClimbMinRot = 0;
    public static final double kL1ClimbMaxRot = 10;
    public static final double MaxClimbMotorVelocity = 45;
    public static final double MinClimbMotorPosition = 42; 
    public static final double MaxClimbMotorPosition = 85;

    public class L1ClimberPID {
        public static final double P = 1;
        public static final double I = 0;
        public static final double D = 0;
        public static final double FF = 0;
    }

    public class Slot0Configs {
        public static final double kP = 0.12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kS = 0.20;
        public static final double kMaxSpeed = 80;
    }

}
