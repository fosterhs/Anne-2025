package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.VelocityVoltage;

public class AlgaeYeeter {
  private final TalonFX armMotor = new TalonFX(13, "canivore"); // Initializes the motor with CAN ID of 0 connected to the canivore. 
  private final TalonFX intakeMasterMotor = new TalonFX(14, "canivore");  // Initializes the motor with CAN ID of 0 connected to the canivore. 
  private final TalonFX intakeSlaveMotor = new TalonFX(15, "canivore");  // Initializes the motor with CAN ID of 0 connected to the canivore. 
  private final DigitalInput algaeSensor = new DigitalInput(4); // Initializes the sensor connected to DIO port 4 on the RoboRIO.

  public AlgaeYeeter() {
    configArmMotor(armMotor, false, 120.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 
    configIntakeMotor(intakeMasterMotor, false, 120.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 
    configIntakeMotor(intakeSlaveMotor, true, 120.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 
    armMotor.setPosition(0.0, 0.03); // Sets the position of the motor to 0 on startup.
    intakeSlaveMotor.setControl(new Follower(14, true)); // Sets the second intake motor to follow the first intake motor exactly.
  }

  // Sets the arm to the desired angle in degrees.
  public void setArmAngle(double angle) {
    armMotor.setControl(new MotionMagicTorqueCurrentFOC(angle)); // Sets the position of the motor in shaft rotations.
  }

  // Sets the speed of the intake wheels. 1.0 corresponds to full outtake speed, -1.0 is corresponds to full intake speed, and 0.0 is stopped.
  public void setWheelSpeed(double speed) {
    intakeMasterMotor.setControl(new VelocityVoltage(speed).withEnableFOC(true)); // Sets the velocity of the motor in rotations per second.
  }

  // Returns true if an algae is detected by the sensor.
  public boolean algaeDetected() {
    return !algaeSensor.get();
  }

  // Returns the current angle of the arm. 
  public double getArmAngle() {
    return armMotor.getPosition().getValueAsDouble();
  }

  // Updates the SmartDashboard with information about the algae yeeter.
  public void updateDash() {
    //SmartDashboard.putBoolean("Algae Yeeter algaeDetected", algaeDetected());
    //SmartDashboard.putNumber("Algae Yeeter getArmAngle", getArmAngle());
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

  private void configIntakeMotor(TalonFX motor, boolean invert, double currentLimit) {
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