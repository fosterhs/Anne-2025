package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Elevator {
  private final TalonFX elevatorMasterMotor = new TalonFX(9, "canivore"); // The master elevator motor.
  private final TalonFX elevatorSlaveMotor = new TalonFX(10, "canivore"); // The slave elevator motor.
  public enum Level {L1, L2, L3, L4, Source, Bottom}   // A list containing important elevator heights that are pre-programmed.
  private final double highLimit = 56.0; // The high limit of the elevator motor in meters.
  private final double lowLimit = 1.0; // The low limit of the elevator motor in meters.
  private final double posTol = 0.1; // How much error is acceptable between the setpoint and the current position of the elevator in motor rotations.
  private double setpoint = 0.0; // The position that the elevator motor is trying to reach in motor rotations.

  public Elevator() {
    configMotor(elevatorMasterMotor, false, 120.0); // Configures the motor with counterclockwise rotation positive and 25A current limit.
    configMotor(elevatorSlaveMotor, true, 120.0); // Configures the motor with clockwise rotation positive and 25A current limit.
    elevatorMasterMotor.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorSlaveMotor.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorSlaveMotor.setControl(new Follower(9, true)); // Sets the slave motor to follow the master motor exactly.
  }

  // Sets the elevator to a pre-programmed position. 
  public void setLevel(Level desiredLevel) {
    switch(desiredLevel) {
      case L1:
        setPosition(9.95);
      break;

      case L2:
        setPosition(19.46); 
      break;

      case L3:
        setPosition(37.64);
      break;
      
      case L4:
        setPosition(78.31);
      break;

      case Source:
        setPosition(4.75);
      break;

      case Bottom:
        setPosition(0.0);
      break;
    }
  }

  // Checks if the motor is at the target position.
  public boolean atSetpoint() {
    return Math.abs(elevatorMasterMotor.getPosition().getValueAsDouble() - setpoint) < posTol; // Checks if the motor is at the target position.
  }

  // Returns the current position of the elevator in meters.
  public double getPosition() {
    return (getMasterPosition() + getSlavePosition()) / 2.0;
  }

  // Updates the SmartDashboard with information about the elevator.
  public void updateDash() {
    //SmartDashboard.putNumber("Elevator Master Position", getMasterPosition());
    //SmartDashboard.putNumber("Elevator Slave Position", getSlavePosition());
    //SmartDashboard.putNumber("Elevator Position", getPosition());
    //SmartDashboard.putBoolean("Elevator AtSetpoint", atSetpoint());
    //SmartDashboard.putNumber("Elevator Setpoint", setpoint);
  }

  // Sets the position of the elevator motor in meters.
  private void setPosition(double position) {
    if (position > highLimit) position = highLimit; // If the position is greater than the high limit, set the position to the high limit.
    if (position < lowLimit) position = lowLimit; // If the position is less than the low limit, set the position to the low limit.
    elevatorMasterMotor.setControl(new MotionMagicTorqueCurrentFOC(position)); 
  }
  
  // Returns the position of the master elevator motor in meters.
  private double getMasterPosition() {
    return elevatorMasterMotor.getPosition().getValueAsDouble();
  }

  // Returns the position of the slave elevator motor in meters.
  private double getSlavePosition() {
    return elevatorSlaveMotor.getPosition().getValueAsDouble();
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
    motorConfigs.Slot0.kP = 18.0; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 1.3; // Units: amperes per 1 rotartion / 1 second of error.
    motorConfigs.Slot0.kG = 7.5; // output to overcome gravity
    motorConfigs.Slot0.kS = 3.5; // Units: amperes.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Units: rotations per second.
  
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}