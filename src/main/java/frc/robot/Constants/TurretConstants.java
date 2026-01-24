package frc.robot.Constants;

public class TurretConstants {

    public static final int turretID = 10;
    public static final int shooterID = 12;
    public static final int encoderID = 11;

    public static final double leftLimit = 0;
    public static final double rightLimit = 1;

    public static final double dutyCycleTurret = 0.5;
    public static final double dutyCycleShooter = 0.5;

    public static final double currentLimitTurret = 80;
    public static final double currentLimitShooter = 80;

    public static final double KPTurret = 0.153; //found in simulation
    public static final double KITurret = 0;
    public static final double KDTurret = 0;

    public static final double KPShooter = 0.1; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;
}
