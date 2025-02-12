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
  private final TalonFX spitMotor = new TalonFX(12, "rio");  // Initializes the motor with CAN ID of 12 connected to the canivore. 
  private final DigitalInput coralSensor = new DigitalInput(2); // Initializes the sensor connected to DIO port 2 on the RoboRIO.
  private final Timer coralTimer = new Timer(); // Keeps track of how long coral has not been detected for. Resets to 0 seconds as soon as coral is detected.
  private boolean isSpitting = false; // Returns true if the spitter is in the process of ejecting a coral. 
  private double spitDelay = 0.5; // How long the spitMotor will continue running for after a coral is no longer detected in seconds.

  public CoralSpitter() {
    configMotor(spitMotor, false, 80.0); // Configures the motor with counterclockwise rotation positive and 80A current limit.
  }

  // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
  public void init() { 
    isSpitting = false;
    coralTimer.restart();
  }

  // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
  public void periodic() { 
    if (coralDetected()) coralTimer.restart();

    if (isSpitting) {
      spitMotor.setControl(new VelocityVoltage(10.0).withEnableFOC(true)); // Sets the velocity of the motor in rotations per second.
    } else {
      spitMotor.setControl(new VelocityVoltage(0.0).withEnableFOC(true)); // Sets the velocity of the motor in rotations per second.
    }

    if (coralTimer.get() > spitDelay) isSpitting = false;
  }
  
  // Tells the coralSpitter to begin the process of ejecting a coral. 
  public void spit() { 
    isSpitting = coralDetected();
  }

  // Returns true if there is a coral detected in the coralSpitter.
  public boolean coralDetected() {
    return !coralSensor.get();
  }

  // Returns true if the coralSpitter is in the process of ejecting a coral.
  public boolean isSpitting() {
    return isSpitting;
  }

  // Updates the SmartDashboard with information about the coralSpitter.
  public void updateDash() { 
    SmartDashboard.putBoolean("coralDetected", coralDetected());
    SmartDashboard.putBoolean("isSpitting", isSpitting);
    SmartDashboard.putNumber("coralTimer", coralTimer.get());
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