package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase{
    // declares motors
    private TalonFX m_turret;
    private TalonFX m_shooter;
    private CANcoder m_encoder;

    //used to control speed of motors
    private DutyCycleOut m_dutyCycleTurret;
    private DutyCycleOut m_dutyCycleShooter;
    private PIDController m_pidControl;

    //limits brownouts by limiting current applied to motors
    private CurrentLimitsConfigs m_turretLimitConfig = new CurrentLimitsConfigs();
    private CurrentLimitsConfigs m_shooterLimitConfig = new CurrentLimitsConfigs();

    // declares x and y offsets from limelight
    private double TX, TY;
    //weather or not limelight sees apriltag
    private boolean TV; 

    private TurretState turretAction;

    public Turret(){
        m_turret = new TalonFX(TurretConstants.turretID);
        m_encoder = new CANcoder(TurretConstants.encoderID);

        // turret motor will start with half speed
        m_dutyCycleTurret = new DutyCycleOut(0.5);
        m_dutyCycleShooter = new DutyCycleOut(1);
        m_pidControl = new PIDController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD);
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        // turretConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        // assigns PID values to shooter for precise speed 
        shooterConfig.Slot0.kP = ShooterConstants.KP; // controls position error
        shooterConfig.Slot0.kI = ShooterConstants.KI; // controls integral error using kP and kD (don't change)
        shooterConfig.Slot0.kD = ShooterConstants.KD; // controls derivative error 

        TalonFXConfigurator turretConfigurator = m_turret.getConfigurator();
        TalonFXConfigurator shooterConfigurator = m_shooter.getConfigurator();       

        m_turret.getConfigurator().apply(turretConfig);
        m_turretLimitConfig.StatorCurrentLimit = TurretConstants.currentLimit;
        m_turretLimitConfig.StatorCurrentLimitEnable = true;
        turretConfigurator.apply(m_turretLimitConfig);

        m_turret.getConfigurator().apply(shooterConfig);
        m_shooterLimitConfig.StatorCurrentLimit = ShooterConstants.currentLimit;
        m_shooterLimitConfig.StatorCurrentLimitEnable = true;
        shooterConfigurator.apply(m_shooterLimitConfig);

        // apply a deadband to turn the turret (in degrees)
        m_pidControl.setTolerance(2);

        TX = VisionConstants.tx;
        TY = VisionConstants.ty;
        TV = VisionConstants.tv;
    }

    public void periodic(){
        TX = LimelightHelpers.getTX("Limelight 4");
        TY = LimelightHelpers.getTY("Limelight 4");
        TV = LimelightHelpers.getTV("Limelight 4");

        //constantly updates turret state
        setTurretAction(turretAction);
    }

    public enum TurretState{
        IDLE, // doesn't move the turret
        MANUAL, // allows custom control of the turret from the controller
        LOCK_ONTO_TARGET, // centers turret to apriltag once
        TRACK // centers turret to apriltag as long as remains in this state
    }

    /**
     * assigns a speed to run the turret motor
     * @param speed
     */
    public void spinTurret(double speed){
        m_turret.setControl(m_dutyCycleTurret.withOutput(speed));
    }

    /**
     * assigns a speed to run the shooter motor
     * @param speed
     */
    public void startShooter(double speed){
        m_shooter.setControl(m_dutyCycleShooter.withOutput(speed));
    }

    /**
     * stops both the turret and shooter movement 
     * brake mode keeps turret from excess movement 
     */
    public void stopMotors(){
        m_turret.setControl(m_dutyCycleTurret.withOutput(0));
        m_shooter.setControl(m_dutyCycleTurret.withOutput(0));
        m_turret.setNeutralMode(NeutralModeValue.Brake);
    }

    // finds the current angle of the turret with an external encoder
    public double getTurretDegree(){
        double angle = m_encoder.getAbsolutePosition().getValueAsDouble();
        return angle;
    }

    /**
     * allows the operator to control the direction of the turret
     * reverses the speed once the turret reaches left or right limits
     * @param speed
     */
    public void rotateManual(double speed){
        if(ControllerConstants.operatorController.getLeftTriggerAxis() > 0.2){
            double limit = TurretConstants.leftLimit;

            if(getTurretDegree() >= limit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(speed));

            } else if(getTurretDegree() <= limit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(-speed));

            }
        } else if(ControllerConstants.operatorController.getRightTriggerAxis() > 0.2){
            double limit = TurretConstants.rightLimit;

            if(getTurretDegree() <= limit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(speed));

            } else if(getTurretDegree() >= limit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(-speed));

            }
        } else{
            m_turret.setControl(m_dutyCycleTurret.withOutput(0));
        }
    }

    /**
     *  . <- april tag height
     *  | \
     *  |  \ 
     *  |__(\. <- limelight camera height
     *   ^
     *   |
     * @return length from camera to apriltag
     */
    public double findDistance(){
        double degreesToGoal = VisionConstants.mountedDegree + TY;
        double radiansToGoal = Math.toRadians(degreesToGoal);

        return (VisionConstants.hubApriltagHeight - VisionConstants.lensHeight)/Math.tan(radiansToGoal);
    }

    /**
     * @return degree in radians that robot needs to turn to get to center of apriltag if apritag is detected
     * @return no movement if apriltag is not found
     */
    public double findAngleToTarget(){
        if(TV == true){
            double distance = findDistance();
            double xRadians = Math.toRadians(TX); //true using this as angle to target if no offset is given

            //  double xTarget = VisionConstants.horizontalOffset + distance*Math.sin(xRadians);
            //  double yTarget = VisionConstants.verticalOffset + distance*Math.cos(xRadians);

            //  double angle = Math.atan2(yTarget, xTarget);
            return xRadians;
        } else{
            System.out.println("ERROR: APRILTAG NOT FOUND");
            return 0; 
        }
    }

    double limelight_aim_proportional(){    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= kP;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    } 

    /**
     * spins turret (in respect to the limit) until apriltag is in range of limelight's vision
     * finds the angle needed to center the turret to the apriltag and turns the turret by the desired angle
     * @param speed 
     */
    public void turnUntilApriltag(double speed){
        if(!TV){
            m_turret.setControl(m_dutyCycleTurret.withOutput(speed));

            if(getTurretDegree() >= TurretConstants.leftLimit || getTurretDegree() <= TurretConstants.rightLimit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(-speed));
            }
        } else{
            stopMotors();
            m_pidControl.setSetpoint(findAngleToTarget());
            spinTurret(m_pidControl.calculate(getTurretDegree()));

            if(m_pidControl.atSetpoint()){
                turretAction = TurretState.IDLE;
            }
        }
    }

    /**
     * spins turret (in respect to the limit) until apriltag is in range of limelight's vision
     * contiously centers turret to the apriltag 
     * @param speed
     */
    public void track(double speed){
        if(!TV){
            m_turret.setControl(m_dutyCycleTurret.withOutput(speed));

            if(getTurretDegree() >= TurretConstants.leftLimit || getTurretDegree() <= TurretConstants.rightLimit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(-speed));
            }
        } else{
            stopMotors();
            m_pidControl.setSetpoint(findAngleToTarget());
            spinTurret(m_pidControl.calculate(getTurretDegree()));
        }
    }

    /**
     * assigns prior functions to each state of the turret
     * @param state
     */
    public void setTurretAction(TurretState state){
        switch (state){
            case IDLE:        stopMotors();              break;
            case MANUAL:      rotateManual(0.5);        break;
            case FIND_TARGET: turnUntilApriltag(0.1);   break;
            case TRACK:       track(0.1);          break; 
            default: stopMotors();                       break;
        }
    }
}
