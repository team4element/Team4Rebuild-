package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Turret extends SubsystemBase{
TalonFX m_turret;
CANcoder m_encoder;

DutyCycleOut m_dutyCycle;
CurrentLimitsConfigs m_limitConfig = new CurrentLimitsConfigs();

double TX, TY;
boolean TV;

TurretState turretAction;
    
    public Turret(){
        m_turret = new TalonFX(TurretConstants.turretID);
        m_encoder = new CANcoder(TurretConstants.encoderID);
        m_dutyCycle = new DutyCycleOut(0.5); //motor will start with half speed
        TalonFXConfiguration config = new TalonFXConfiguration();
      //  config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        config.Slot0.kP = 1;
        config.Slot0.kI = 1;
        config.Slot0.kD = 1;

        TalonFXConfigurator configurator = m_turret.getConfigurator();

        m_turret.getConfigurator().apply(config);
        m_limitConfig.StatorCurrentLimit = TurretConstants.currentLimit;
        m_limitConfig.StatorCurrentLimitEnable = true;
        configurator.apply(m_limitConfig);

        TX = VisionConstants.tx;
        TY = VisionConstants.ty;
        TV = VisionConstants.tv;
    }

    public void periodic(){
        TX = LimelightHelpers.getTX("Limelight 4");
        TY = LimelightHelpers.getTY("Limelight 4");
        TV = LimelightHelpers.getTV("Limelight 4");

        setTurretAction(turretAction);
    }

    public enum TurretState{
        IDLE, MANUAL, FIND_TARGET, TRACK
        //find target gets in range to see apriltag

    }

    public void runMotor(double speed){
        m_turret.setControl(m_dutyCycle.withOutput(speed));
    }

    public void stopMotor(){
        m_turret.setControl(m_dutyCycle.withOutput(0));
        m_turret.setNeutralMode(NeutralModeValue.Brake);
    }

    public void turnUntilApriltag(){
        if(!TV){
        m_turret.setControl(m_dutyCycle.withOutput(0.1));
        } else{
            stopMotor();
        }
    }

    public double getTurretDegree(){//finds the angle with an external encoder
        double angle = m_encoder.getAbsolutePosition().getValueAsDouble();

        return angle;
    }

    public void rotateManual(double speed){
        if(ControllerConstants.operatorController.getLeftTriggerAxis() > 0.2){
            double limit = TurretConstants.leftLimit;

            if(getTurretDegree() >= limit){
                m_turret.setControl(m_dutyCycle.withOutput(speed));

            } else if(getTurretDegree() <= limit) {
                m_turret.setControl(m_dutyCycle.withOutput(-speed));

            }
        } else if (ControllerConstants.operatorController.getRightTriggerAxis() > 0.2){
            double limit = TurretConstants.rightLimit;

            if(getTurretDegree() <= limit){
                m_turret.setControl(m_dutyCycle.withOutput(speed));

            } else if(getTurretDegree() >= limit) {
                m_turret.setControl(m_dutyCycle.withOutput(-speed));

            }
        } else {
            m_turret.setControl(m_dutyCycle.withOutput(0));
        }

    }

    public double findDistance(){
        double degreesToGoal = VisionConstants.mountedDegree + TY;
        double radiansToGoal = Math.toRadians(degreesToGoal);

        return (VisionConstants.hubApriltagHeight - VisionConstants.lensHeight)/Math.tan(radiansToGoal);
    }

    public double findAngleToTarget(){
        if (TV == true){
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

    public void setTurretAction(TurretState state){
        switch (state) {
            case IDLE:        stopMotor();              break;
            case MANUAL:      rotateManual(0.5);  break;
            case FIND_TARGET: turnUntilApriltag();      break;
            case TRACK:       findAngleToTarget();      break; 
            default: stopMotor();  break;
        }
    }
   
}
