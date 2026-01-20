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

    // Used to control speed of the motor
    private DutyCycleOut m_dutyCycle;
    // Used to control position of the motor
    private PositionVoltage m_positionRequest;
    private CurrentLimitsConfigs m_currentLimit;

    // Controls print statement output to terminal
    private boolean debug;

    public Climb(){
        m_motor = new TalonFX(ClimbConstants.ID);

        m_dutyCycle = new DutyCycleOut(0.5);
        m_currentLimit = new CurrentLimitsConfigs();

        m_currentLimit.StatorCurrentLimit = ClimbConstants.currentLimit;
        m_currentLimit.StatorCurrentLimitEnable = true;

        debug = false; // This should be true when testing
    }

    // Constantly gets the motor's rotation while testing.
    public void periodic(){
        if(debug == true){
        double currentRotation = m_motor.getPosition().getValueAsDouble();
        System.out.println("CLIMB POSITION: " + currentRotation); 

        }
    }

    /*
     * Initiates motor starting position.
     */
    public void resetMotor(){
        m_motor.setPosition(0);
    }

    /*
     * Stops the motor and keeps the climb at current position.
     */
    public void hold(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
        m_motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs the motor in reverse to pull the robot up until it reaches the hook's physical limit.
     * @param speedPercentage from -1 to 1.
     */
    public void runMotorDown(double speedPercentage){
        if(m_motor.getPosition().getValueAsDouble() == lowLimit){
            hold();

        } else{
            m_motor.setControl(m_dutyCycle.withOutput(-speedPercentage));
        }
    }

    /**
     * Runs the motor to reach to the next level until it reaches the hook's physical limit.
     * @param speedPercentage from -1 to 1.
     */
    public void runMotorUp(double speedPercentage){
        if(m_motor.getPosition().getValueAsDouble() == highLimit){
            hold();

        } else{
            m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
        }
    }

    /**
     * Assigns a goal value for the climb position to reach next level. 
     * @param setpoint based on motor rotation.
     */
    public void goToSetpoint(double setpoint){
        m_motor.setControl(m_positionRequest.withPosition(setpoint));
    }
}
