package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSpitter {
  private final TalonFX spitMotor = new TalonFX(12, "rio");  // Initializes the motor with CAN ID of 12 connected to the canivore. 
  private final DigitalInput coralIntakeSensor = new DigitalInput(3); // Initializes the sensor connected to DIO port 3 on the RoboRIO. Sensor 1 is the sensor closest to the intake.
  private final DigitalInput coralExhaustSensor = new DigitalInput(2); // Initializes the sensor connected to DIO port 2 on the RoboRIO. Sensor 2 is the sensor closest to the exhaust.
  private final Timer intakeSensorTimer = new Timer(); // Keeps track of how long coral has not been detected for. Resets to 0 seconds as soon as coral is detected.
  private final Timer exhaustSensorTimer = new Timer(); // Keeps track of how long coral has not been detected for. Resets to 0 seconds as soon as coral is detected.
  private boolean isSpitting = false; // Returns true if the spitter is in the process of ejecting a coral. 
  private double exhaustDelay = 0.5; // How long the spitMotor will continue running for after a coral is no longer detected in seconds.
  private double intakeDelay = 0.5; // How long the spitMotor will wait before starting to intake a coral in seconds.

  public CoralSpitter() {
    configMotor(spitMotor, false, 120.0); // Configures the motor with counterclockwise rotation positive and 80A current limit.
    exhaustSensorTimer.restart();
    intakeSensorTimer.restart();
    ParentDevice.optimizeBusUtilizationForAll(spitMotor);
  }

  // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
  public void init() { 
    isSpitting = false;
  }

  // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
  public void periodic() {
    if (getExhaustSensor()) exhaustSensorTimer.restart(); // Restarts the timer as soon as a coral is detected. The timer measures how much time has elapsed since a coral was last detected.
    if (!getIntakeSensor()) intakeSensorTimer.restart(); // Restarts the timer as soon as a coral is not detected. The timer measures how much time has elapsed after a coral is detected.

    if (isSpitting) {
      spitMotor.setControl(new VoltageOut(1.8).withEnableFOC(true)); // Scores the coral
    } else if (!getExhaustSensor() && intakeSensorTimer.get() > intakeDelay) {
      spitMotor.setControl(new VoltageOut(1.2).withEnableFOC(true)); // Loads the coral about halfway into the mechanism.
    } else if (!getExhaustSensor()) {
      spitMotor.setControl(new VoltageOut(-0.6).withEnableFOC(true)); // Runs the mechanism in reverse to prevent jams.
    } else {
      spitMotor.setControl(new VoltageOut(0.0).withEnableFOC(true)); // Holds the coral until it is ready to be scored.
    }

    if (exhaustSensorTimer.get() > exhaustDelay) isSpitting = false; // If the timer exceeds the delay, stop spitting.
  }
  
  // Tells the coralSpitter to begin the process of ejecting a coral. 
  public void spit() { 
    isSpitting = getExhaustSensor();
  }

  // Returns true if there is a coral detected in the coralSpitter.
  public boolean coralDetected() {
    return getExhaustSensor() || getIntakeSensor(); 
  }

  // Returns true if the intake sensor detects a coral.
  public boolean getIntakeSensor() {
    return !coralIntakeSensor.get();
  }

  // Returns true if the exhaust sensor detects a coral.
  public boolean getExhaustSensor() {
    return !coralExhaustSensor.get();
  }

  // Returns true if the coralSpitter is in the process of ejecting a coral.
  public boolean isSpitting() {
    return isSpitting;
  }

  // Updates the SmartDashboard with information about the coralSpitter.
  public void updateDash() { 
    //SmartDashboard.putBoolean("Spitter getIntakeSensor", getIntakeSensor());
    //SmartDashboard.putBoolean("Spitter getExhaustSensor", getExhaustSensor());
    //SmartDashboard.putBoolean("Spitter coralDetected", coralDetected());
    //SmartDashboard.putBoolean("Spitter isSpitting", isSpitting);
    //SmartDashboard.putNumber("Spitter Intake Timer", intakeSensorTimer.get());
    //SmartDashboard.putNumber("Spitter Exhaust Timer", exhaustSensorTimer.get());
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