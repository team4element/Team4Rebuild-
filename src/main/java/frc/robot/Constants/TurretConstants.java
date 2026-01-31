package frc.robot.Constants;

public class TurretConstants {

    public static final int turretID = 19;
    public static final int shooterID = 20;
    public static final int encoderID = 11;

    public static final double leftLimit = 90;
    public static final double rightLimit = -90;

    public static final double dutyCycleTurret = 0.5;
    public static final double dutyCycleShooter = 0.5;

    public static final double currentLimitTurret = 80;
    public static final double currentLimitShooter = 80;

    public static final double KPTurret = 0.16; //found in simulation 0.153 (power)
    public static final double KITurret = 0.02;
    public static final double KDTurret = 0; //damp, helps with ossilation, if too jerky -> higher value, if too slow -> lower value + more kP

    public static final double KPShooter = 0.5; 
    public static final double KIShooter = 0;
    public static final double KDShooter = 0;

    public static final double shooterSpeed = 0.5;
    public static final double shooterTimeout = 2;

    public static final double turretSpeed = 0.2;
}
