package frc.robot;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralSpitter {
  private final TalonFXS spitMotor = new TalonFXS(12, "rio");  // Initializes the motor with CAN ID of 12 connected to the roboRIO.
  private final VoltageOut spitMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Communicates voltage requests to the spit motor.
  private final DigitalInput coralIntakeSensor = new DigitalInput(0); // Initializes the sensor connected to DIO port 0 on the RoboRIO. Sensor 1 is the sensor closest to the intake.
  private final DigitalInput coralExhaustSensor = new DigitalInput(1); // Initializes the sensor connected to DIO port 1 on the RoboRIO. Sensor 2 is the sensor closest to the exhaust.
  private final Timer intakeSensorTimer = new Timer(); // Keeps track of how long coral has not been detected for. Resets to 0 seconds as soon as coral is detected.
  private final Timer exhaustSensorTimer = new Timer(); // Keeps track of how long coral has not been detected for. Resets to 0 seconds as soon as coral is detected.
  private final double exhaustDelay = 0.3; // How long the spitMotor will continue running for after a coral is no longer detected in seconds.
  private final double intakeDelay = 0.3; // How long the spitMotor will wait before starting to intake a coral in seconds.
  private boolean isSpitting = false; // Returns true if the spitter is in the process of ejecting a coral. 

  public CoralSpitter() {
    configMotor(spitMotor, true); // Configures the motor.
    ParentDevice.optimizeBusUtilizationForAll(spitMotor);
    exhaustSensorTimer.restart();
    intakeSensorTimer.restart();
  }

  // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
  public void init() { 
    isSpitting = false;
  }

  // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
  public void periodic() {
    if (getExhaustSensor()) exhaustSensorTimer.restart(); // Restarts the timer as soon as a coral is detected. The timer measures how much time has elapsed since a coral was last detected.
    if (!getIntakeSensor()) intakeSensorTimer.restart(); // Restarts the timer as soon as a coral is not detected. The timer measures how much time has elapsed after a coral is detected.
    if (exhaustSensorTimer.get() > exhaustDelay) isSpitting = false; // If the timer exceeds the delay, stop spitting.

    if (isSpitting) {
      spitMotor.setControl(spitMotorVoltageRequest.withOutput(12.0)); // Scores the coral
    } else if (!getExhaustSensor() && intakeSensorTimer.get() > intakeDelay) {
      spitMotor.setControl(spitMotorVoltageRequest.withOutput(1.8)); // Loads the coral about halfway into the mechanism.
    } else if (!getExhaustSensor()) {
      spitMotor.setControl(spitMotorVoltageRequest.withOutput(-0.9)); // Runs the mechanism in reverse to prevent jams.
    } else {
      spitMotor.setControl(spitMotorVoltageRequest.withOutput(0.0)); // Holds the coral until it is ready to be scored.
    }
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

  // Returns the current value of the exhaust timer
  public double getExhaustTimer() {
    return exhaustSensorTimer.get();
  }

  // Returns the current value of the intake timer
  public double getIntakeTimer() {
    return intakeSensorTimer.get();
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
    //SmartDashboard.putBoolean("Spitter isSpitting", isSpitting());
    //SmartDashboard.putNumber("Spitter Intake Timer", getIntakeTimer());
    //SmartDashboard.putNumber("Spitter Exhaust Timer", getExhaustTimer());
  }

  private void configMotor(TalonFXS motor, boolean invert) {
    TalonFXSConfiguration motorConfigs = new TalonFXSConfiguration();

    motorConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    motorConfigs.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;

    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(motorConfigs, 0.03);
  }      
}