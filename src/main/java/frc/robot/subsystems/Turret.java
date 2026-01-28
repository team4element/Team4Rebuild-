/*
 * This subsystem should track the apriltag using limelight camera data by spinning the turret and shoot fuel (scoring element) into the hub (score)
 * The turret's actions are given by states: IDLE, MANUAL, LOCK_ONTO_TARGET, TRACK_APRILTAG
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase{
    public enum TurretState{
        IDLE, // Doesn't move the turret
        MANUAL, // Allows custom control of the turret from the controller
        LOCK_ONTO_TARGET, // Centers the turret to apriltag once
        TRACK_APRILTAG // Centers the turret to apriltag as long as it remains in this state
    }

    // Declares motors and sensors
    private TalonFX m_turret, m_shooter;
    private CANcoder m_encoder;

    // Used to control speed of motors
    private DutyCycleOut m_dutyCycleTurret, m_dutyCycleShooter;
    private PositionVoltage m_positionRequest;
    private PIDController m_pidControl;
    private VelocityVoltage m_voltage;

    // Limits brownouts by limiting current applied to motors
    private CurrentLimitsConfigs m_turretLimitConfig, m_shooterLimitConfig;

    // Declares x and y offsets from limelight
    private double TX, TY;
    // Determines weather or not limelight sees apriltag
    private boolean TV; 

    private TurretState turretAction;

    public Turret(){
        m_turret = new TalonFX(TurretConstants.turretID);
        m_shooter = new TalonFX(TurretConstants.shooterID);
        m_encoder = new CANcoder(TurretConstants.encoderID);

        // The turret and shooter motor will start with half speed
        m_dutyCycleTurret = new DutyCycleOut(TurretConstants.dutyCycleTurret);
        m_dutyCycleShooter = new DutyCycleOut(TurretConstants.dutyCycleShooter);
        m_pidControl = new PIDController(TurretConstants.KPTurret, TurretConstants.KITurret, TurretConstants.KDTurret);

        m_positionRequest = new PositionVoltage(0);
        m_voltage = new VelocityVoltage(0).withSlot(0);

        m_turretLimitConfig = new CurrentLimitsConfigs();
        m_shooterLimitConfig = new CurrentLimitsConfigs();

        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        // Assigns PID values to the shooter for precise speed 
        //shooterConfig.Slot0.kP = TurretConstants.KPTurret; // Controls position error
        //shooterConfig.Slot0.kI = TurretConstants.KITurret; // Controls integral error using kP and kD (don't change)
        //shooterConfig.Slot0.kD = TurretConstants.KDTurret; // Controls derivative error 

        // Assigns PID values to the turret for precise speed 
        turretConfig.Slot0.kP = TurretConstants.KPTurret; // Controls position error
        turretConfig.Slot0.kI = TurretConstants.KITurret; // Controls integral error using kP and kD (don't change)
        turretConfig.Slot0.kD = TurretConstants.KDTurret; // Controls derivative error 

        TalonFXConfigurator turretConfigurator = m_turret.getConfigurator();
        TalonFXConfigurator shooterConfigurator = m_shooter.getConfigurator();       

        m_turret.getConfigurator().apply(turretConfig);
        m_turretLimitConfig.StatorCurrentLimit = TurretConstants.currentLimitTurret;
        m_turretLimitConfig.StatorCurrentLimitEnable = true;
        turretConfigurator.apply(m_turretLimitConfig);

       // m_turret.getConfigurator().apply(shooterConfig);
       // m_shooterLimitConfig.StatorCurrentLimit = TurretConstants.currentLimitShooter;
       // m_shooterLimitConfig.StatorCurrentLimitEnable = true;
       // shooterConfigurator.apply(m_shooterLimitConfig);

        // Applies a deadband to turn the turret (in degrees)
        // TODO: replace parameter to a constant
        m_pidControl.setTolerance(2);

        TX = VisionConstants.tx;
        TY = VisionConstants.ty;
        TV = VisionConstants.tv;

        turretAction = TurretState.IDLE;
    }

    /**
     * Assigns a speed to run the turret motor.
     * @param speedPercentage from -1 to 1.
     */
    public void spinTurret(double speedPercentage){
        m_turret.set(m_pidControl.calculate(speedPercentage));
    }

    /*
     * Sets the turret's starting position
     */
    public void resetTurret(){
        m_turret.setPosition(0);
    }

    /**
     * Assigns a speed to run the shooter motor.
     * @param speedPercentage from -1 to 1.
     */
    public void startShooter(double speedPercentage){
        //m_shooter.setControl(m_voltage.withVelocity(speedPercentage).withFeedForward(0.5));
        m_shooter.setControl(m_dutyCycleShooter.withOutput(speedPercentage));
    }

    /**
     * Stops both the turret and shooter movement.  
     */
    public void stopMotors(){
   //     m_turret.setControl(m_dutyCycleTurret.withOutput(0));
    //    m_shooter.setControl(m_dutyCycleTurret.withOutput(0));
        m_turret.setControl(m_dutyCycleTurret.withOutput(0));
        m_shooter.setControl(m_dutyCycleTurret.withOutput(0));
      //  m_turret.setNeutralMode(NeutralModeValue.Brake);
    }

    /*
     * Moves the turret to it's 0 position (facing forward)
     */
    public void returnToStartPosition(){
        m_turret.setControl(m_positionRequest.withPosition(0));
    }

    /**
     * Finds the current angle in degrees of the turret with an external encoder.
     * @return the angle.
     */ 
    public double getTurretDegree(){
        double angle = m_encoder.getAbsolutePosition().getValueAsDouble();
        return angle;
    }

    public double getMotorRotation(){
        return m_turret.getPosition().getValueAsDouble();
    }

    /**
     * Allows the operator to control the direction of the turret.
     * Reverses the speed once the turret reaches the left or right limits.
     * @param speedPercentage as a percentage from -1 to 1.
     */
    public void rotateManual(double speedPercentage){
        if(ControllerConstants.operatorController.x().getAsBoolean()){
            double limit = TurretConstants.leftLimit;

            if(getMotorRotation() <= limit){
                m_turret.set(speedPercentage);

            } else if(getMotorRotation() >= limit){
                m_turret.set(-speedPercentage);

            }
        } else if(ControllerConstants.operatorController.b().getAsBoolean()){
            double limit = TurretConstants.rightLimit;

            if(getMotorRotation() >= limit){
                m_turret.set(-speedPercentage);

            } else if(getMotorRotation() <= limit){
                m_turret.set(speedPercentage);

            }
        } else{
            m_turret.set(0);
        }
    }

    /**
     * Finds the distance between the apriltag and the limelight (center of lens) 
     * @link https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance  -> Explains the calculations to find the distance
     * @return The distance as a degree in radians
     */
  /* public double findDistance(){
        double degreesToGoal = VisionConstants.mountedDegree + TY;
        double radiansToGoal = Math.toRadians(degreesToGoal);

        return (VisionConstants.hubApriltagHeight - VisionConstants.lensHeight)/Math.tan(radiansToGoal);
    }
        */

    /**
     * Gets the degree the robot needs to turn to get to center of apriltag if the apritag is detected.
     * Results in an error if the apriltag isn't there.
     * @return The degree in radians.
     */
    /*
    public double findAngleToTarget(){
        if(TV == true){
            double distanceToApriltag = findDistance();
            double xAngleInRadians = Math.toRadians(distanceToApriltag); // try using this as the angle to target if no x offset is given

            //  double xTarget = VisionConstants.horizontalOffset + distance*Math.sin(xRadians);
            //  double yTarget = VisionConstants.verticalOffset + distance*Math.cos(xRadians);

            //  double angle = Math.atan2(yTarget, xTarget);
            return xAngleInRadians;
        } else{
            int apriltagNotFound = 0;
            System.out.println("ERROR: APRILTAG NOT FOUND");

            return apriltagNotFound; 
        }
    }
        */

    /**
     * Gets the velocity needed to center the turret to the apriltag by the x axis of the turret
     * @return the speed
     */
    /*
    //TODO: This should be tested later with turnUntilApriltag function
    public double limelight_aim_proportional(){    
        // kP (constant of proportionality)
        // This is a hand-tuned number that determines the aggressiveness of our proportional control loop.
        // If it is too high, the robot will oscillate.
        // If it is too low, the robot will never reach its target.
        // If the turret never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = TX * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= kP;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    } 
    */

    /**
     * Spins the turret (in respect to the limit) until the apriltag is in range of the limelight's vision.
     * Finds the angle needed to center the turret to the apriltag and turns the turret by the desired angle.
     * @param speedPercentage as a percetage from -1 to 1.
     */
    /*
    public void turnUntilApriltag(double speedPercentage){
        if(!TV){
            m_turret.setControl(m_dutyCycleTurret.withOutput(speedPercentage));

            if(getTurretDegree() >= TurretConstants.leftLimit || getTurretDegree() <= TurretConstants.rightLimit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(-speedPercentage));
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
        */

    /**
     * Spins the turret (in respect to the limit) until the apriltag is in range of the limelight's vision.
     * Contiously centers the turret to the apriltag. 
     * @param speedPercentage from -1 to 1.
     */
    /*
    public void track(double speedPercentage){
        if(!TV){
            m_turret.setControl(m_dutyCycleTurret.withOutput(speedPercentage));

            if(getTurretDegree() >= TurretConstants.leftLimit || getTurretDegree() <= TurretConstants.rightLimit){
                m_turret.setControl(m_dutyCycleTurret.withOutput(-speedPercentage));
            }
        } else{
            stopMotors();
            m_pidControl.setSetpoint(findAngleToTarget());
            spinTurret(m_pidControl.calculate(getTurretDegree()));
        }
    }
    */

    public Command setTurretState(TurretState state){
        return new InstantCommand(()-> turretAction = state);
    }

    /**
     * Assigns the prior functions to each state of the turret.
     * @param state as listed in TurretState enum.
     */
    public void setTurretAction(TurretState state){
        switch (state){
            case IDLE:              stopMotors();                 break;
            case MANUAL:            stopMotors();            break;
            case LOCK_ONTO_TARGET:  stopMotors();       break;
            case TRACK_APRILTAG:    stopMotors();                   break; 
            default: stopMotors();                                break;
        }
    }

    public void periodic(){
        TX = LimelightHelpers.getTX("Limelight 4");
        TY = LimelightHelpers.getTY("Limelight 4");
        TV = LimelightHelpers.getTV("Limelight 4");

        System.out.println(getMotorRotation());

        // Constantly updates the turret state
       setTurretAction(turretAction);
    }
}
