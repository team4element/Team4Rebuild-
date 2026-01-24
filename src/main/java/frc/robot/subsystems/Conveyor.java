/*
 * This subsystem moves the fuel (game piece) from the spinster into the turret.
 * The conveyor simply runs using a series of rollers.
 */

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ConveyorConstants;

public class Conveyor {
    // Declare the conveyor motor
    private TalonFX m_motor;

    // Controls the motor's output/speed
    private DutyCycleOut m_dutyCycle;
    // Used as an additional limit to the amount of voltage the motor could use that helps prevent brownout
    private CurrentLimitsConfigs m_currentLimit;

    public Conveyor(){
        m_motor = new TalonFX(ConveyorConstants.conveyorID);

        // The motor will start with half speed
        m_dutyCycle = new DutyCycleOut(ConveyorConstants.dutyCycle);
        m_currentLimit = new CurrentLimitsConfigs();

        m_currentLimit.StatorCurrentLimit = ConveyorConstants.currentLimit;
        m_currentLimit.StatorCurrentLimitEnable = true;
    }

    /**
     * Applies a desired power to the motor
     * @param speedPercentage from -1 to 1
     */
    public void runMotor(double speedPercentage){
        m_motor.setControl(m_dutyCycle.withOutput(speedPercentage));
    }

    /*
     * Stops the movement of the motor
     */
    public void stopMotor(){
        m_motor.setControl(m_dutyCycle.withOutput(0));
    }
}
