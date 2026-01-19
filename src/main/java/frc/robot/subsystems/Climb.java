/*
 * This subsystem controls the belt on the climb to pull onto levels 1, 2, and 3
 * The climb can be used manually and by internal encoder position
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase{
    // Declare the climb motor
    private TalonFX m_motor;

    // Declare the vertical bounds based on encoder 
    private double highLimit;
    private double lowLimit;

    private DutyCycleOut m_dutyCycle;
    private PositionVoltage m_positionRequest;
    private CurrentLimitsConfigs m_currentLimit;

    private boolean debug;

    public Climb(){
        m_motor = new TalonFX(ClimbConstants.ID);

        m_dutyCycle = new DutyCycleOut(0.5);
        m_currentLimit = new CurrentLimitsConfigs();

        m_currentLimit.StatorCurrentLimit = ClimbConstants.currentLimit;
        m_currentLimit.StatorCurrentLimitEnable = true;

        debug = false;
    }

    public void periodic(){
        if(debug == true){
        double currentPosition = m_motor.getPosition().getValueAsDouble();
        System.out.println("CLIMB POSITION: " + currentPosition); 
        
        }
    }

    public void resetMotor(){
        m_motor.setPosition(0);
    }

    public void hold(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void manual(double speedPercentage){
        m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
    }

    public void pullDown(double speedPercentage){
        if(m_motor.getPosition().getValueAsDouble() == lowLimit){
            hold();

        } else{
            m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
        }
    }

    public void climbUp(double speedPercentage){
        if(m_motor.getPosition().getValueAsDouble() == highLimit){
            hold();

        } else{
            m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
        }
    }

    public void goToSetpoint(double setpoint){
        m_motor.setControl(m_positionRequest.withPosition(setpoint));
    }
}
