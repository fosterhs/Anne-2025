package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Elevator {
  private final TalonFX elevatorMasterMotor = new TalonFX(9, "canivore"); // The master elevator motor.
  private final TalonFX elevatorSlaveMotor = new TalonFX(10, "canivore"); // The slave elevator motor.
  private final MotionMagicTorqueCurrentFOC elevatorMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Communicates motion magic torque current FOC position requests to the elevator motor.
  private final StatusSignal<Angle> elevatorMasterMotorPosition; // Stores the position of the master elevator motor.
  public enum Level {L1, L2, L3, L4, lowAlgae, highAlgae, bottom, barge} // A list containing important elevator heights that are pre-programmed.
  private final double highLimit = 133.0; // The high limit of the elevator motor in motor rotations.
  private final double lowLimit = 0.5; // The low limit of the elevator motor in motor rotations.
  private final double posTol = 0.5; // How much error is acceptable between the setpoint and the current position of the elevator in motor rotations.
  private double setpoint = 0.0; // The position that the elevator motor is trying to reach in motor rotations.
  private Level currLevel = Level.bottom; // Stores the last commanded position of the arm.

  public Elevator() {
    configMotor(elevatorMasterMotor, true, 120.0); // Configures the motor with counterclockwise rotation positive and 120A current limit.
    configMotor(elevatorSlaveMotor, false, 120.0); // Configures the motor with clockwise rotation positive and 120A current limit.
    elevatorMasterMotor.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorMasterMotorPosition = elevatorMasterMotor.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, elevatorMasterMotorPosition);
    ParentDevice.optimizeBusUtilizationForAll(elevatorMasterMotor, elevatorSlaveMotor);
    elevatorSlaveMotor.setControl(new Follower(elevatorMasterMotor.getDeviceID(), true)); // Sets the slave motor to follow the master motor exactly.
  }

  // Sets the elevator to a pre-programmed position. 
  public void setLevel(Level desiredLevel) {
    switch(desiredLevel) {
      case L1:
        setMotorRotations(highLimit);
        currLevel = Level.L1;
      break;

      case L2:
        setMotorRotations(35.0); 
        currLevel = Level.L2;
      break;

      case L3:
        setMotorRotations(74.0);
        currLevel = Level.L3;
      break;
      
      case L4:
        setMotorRotations(highLimit);
        currLevel = Level.L4;
      break;

      case lowAlgae:
        setMotorRotations(85.0);
        currLevel = Level.lowAlgae;
      break;

      case highAlgae:
        setMotorRotations(6.0);
        currLevel = Level.highAlgae;
      break;

      case bottom:
        setMotorRotations(0.0);
        currLevel = Level.bottom;
      break;

      case barge:
        setMotorRotations(7.0);
        currLevel = Level.barge;
      break;
    }
  }

  // Checks if the motor is at the target position.
  public boolean atSetpoint() {
    return Math.abs(getPosition() - setpoint) < posTol; // Checks if the motor is at the target position.
  }

  // Returns the last requested position of the elevator
  public Level getLevel() {
    return currLevel;
  }

  // Returns the current position of the elevator in motor rotations.
  public double getPosition() {
    return elevatorMasterMotorPosition.refresh().getValueAsDouble();
  }

  // Updates the SmartDashboard with information about the elevator.
  public void updateDash() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putBoolean("Elevator AtSetpoint", atSetpoint());
    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
  }

  // Sets the position of the elevator motor in meters.
  private void setMotorRotations(double desiredRotations) {
    if (desiredRotations > highLimit) desiredRotations = highLimit; // If the position is greater than the high limit, set the position to the high limit.
    if (desiredRotations < lowLimit) desiredRotations = lowLimit; // If the position is less than the low limit, set the position to the low limit.
    elevatorMasterMotor.setControl(elevatorMotorPositionRequest.withPosition(desiredRotations)); 
    setpoint = desiredRotations;
  }

  // Configures the motor with the given parameters.
  private void configMotor(TalonFX motor, boolean invert, double currentLimit) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  
    // Current limit configuration.
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
  
    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 20.0; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 2.0; // Units: amperes per 1 rotartion / 1 second of error.
    motorConfigs.Slot0.kG = 20.0; // output to overcome gravity
    motorConfigs.Slot0.kS = 5.0; // Units: amperes.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Units: rotations per second.
  
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}