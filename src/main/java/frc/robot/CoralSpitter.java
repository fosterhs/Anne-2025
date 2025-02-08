package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSpitter {
  private final TalonFX Coralmotor1 = new TalonFX(12, "canivore");  // Initializes the motor with CAN ID of 0 connected to the canivore. 
  private final DigitalInput sensorSwich = new DigitalInput(3); // Initializes the sensor connected to DIO port 0 on the RoboRIO.

  private final Timer coralTimer = new Timer(); // Keeps track of how long coral has not been detected for. Resets to 0 seconds as soon as coral is detected.

  public CoralSpitter() {
    configMotor(Coralmotor1, false, 80.0); // Configures the motor with counterclockwise rotation positive and 80A current limit.
  }

  public void init() { 

  }

  public void periodic() { 

  }
  
  //Spit out the coral
  public void spit() { 
    Coralmotor1.setControl(new VelocityVoltage(10.0).withEnableFOC(true)); // Sets the velocity of the motor in rotations per second.
    //spiterTimer.reset();
  }

  //uses Senser to see if coral inside Mouth/intake
  public boolean coralDetected() {
    return sensorSwich.get();
  }

  //detect if the spiter is spiting 
  public boolean isSpitting() {
    return true;
  }

  // Updates the SmartDashboard with information about the Spiter.
  public void updateDash() { 
    SmartDashboard.putBoolean("coralIsChewing", coralDetected());
  }

  private void configMotor(TalonFX motor, boolean invert, double currentLimit) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Current limit configuration.
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;

    // VelocityVoltage closed-loop control configuration.
    motorConfigs.Slot0.kP = 0.25; // Units: volts per 1 motor rotation per second of error.
    motorConfigs.Slot0.kI = 0.5; // Units: volts per 1 motor rotation per second * 1 second of error.
    motorConfigs.Slot0.kD = 0.0; // Units: volts per 1 motor rotation per second / 1 second of error.
    motorConfigs.Slot0.kV = 0.12; // The amount of voltage required to create 1 motor rotation per second.
    motorConfigs.Slot0.kS = 0.16; // The amount of voltage required to barely overcome static friction.

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }      
}