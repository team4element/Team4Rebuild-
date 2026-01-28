package frc.robot.Constants;

public class TurretConstants {

    public static final int turretID = 19;
    public static final int shooterID = 20;
    public static final int encoderID = 11;

    public static final double leftLimit = 1.7;
    public static final double rightLimit = -1.7;

    public static final double dutyCycleTurret = 0.5;
    public static final double dutyCycleShooter = 0.5;

    public static final double currentLimitTurret = 80;
    public static final double currentLimitShooter = 80;

    public static final double KPTurret = 0.4; //found in simulation 0.153
    public static final double KITurret = 0;
    public static final double KDTurret = 0;

    public static final double KPShooter = 0.5; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;

    public static final double shooterSpeed = 500;
    public static final double shooterTimeout = 2;

    public static final double turretSpeed = 0.2;
}
