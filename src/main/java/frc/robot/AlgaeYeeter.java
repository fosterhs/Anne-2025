package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class AlgaeYeeter {
  private final TalonFX armMotor1 = new TalonFX(0, "canivore"); // Initializes the motor with CAN ID of 0 connected to the canivore. 
  private final TalonFX mouthMotor1 = new TalonFX(0, "canivore");  // Initializes the motor with CAN ID of 0 connected to the canivore. 

  public AlgaeYeeter() {
    configArmMotor(armMotor1, false, 80.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 
    configMouthMotor1(mouthMotor1, false, 80.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 

    armMotor1.setControl(new MotionMagicTorqueCurrentFOC(0.0)); // Sets the position of the motor in shaft rotations.
    mouthMotor1.setControl(new VelocityVoltage(0.0).withEnableFOC(true)); // Sets the velocity of the motor in rotations per second.
  }

  // Sets the arm to the desired angle in degrees.
  public void setArmAngle(double degree) {

  }

  // Sets the speed of the intake wheels. 1.0 corresponds to full outtake speed, -1.0 is corresponds to full intake speed, and 0.0 is stopped.
  public void setWheelSpeed(double Speed) {

  }

  // Returns true if an algae is detected by the sensor.
  public boolean algaeDetected() {
    return true;
  }

  // Returns the current angle of the arm. 
  public double getArmAngle() {
    return 0.0;
  }

  private void configArmMotor(TalonFX motor, boolean invert, double currentLimit) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  
    // Current limit configuration.
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
  
    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 37.0; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 0.84; // Units: amperes per 1 rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Units: rotations per second.
  
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }

  private void configMouthMotor1(TalonFX motor, boolean invert, double currentLimit) {
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