package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Elevator {
  public final TalonFX elevatorMotor1 = new TalonFX(9, "canivore"); // The master elevator motor.
  private final TalonFX elevatorMotor2 = new TalonFX(10, "canivore"); // The slave elevator motor.
  private final DigitalInput topLimitSwitch = new DigitalInput(0); // Initializes the sensor connected to DIO port 0 on the RoboRIO.
  private final DigitalInput bottomLimitSwitch = new DigitalInput(1); // Initializes the sensor connected to DIO port 1 on the RoboRIO.

  private double setpoint = 0.0; // The position that the elevator motor is trying to reach in motor rotations.
  private double sprocketCircumference = Math.PI * 1.889 * 0.0254; // The diameter of the sprocket in meters.
  private double gearRatio = 12.0; // The gear ratio of the elevator motor.
  private double elevatorRatio = 2.0; // The gear ratio of the elevator motor.
  private double highLimit = 1.31; // The high limit of the elevator motor in meters.
  private double lowLimit = 0.0; // The low limit of the elevator motor in meters.
  private double correctionFactor = 0.92; // The correction factor of the elevator motor.

  // A list containing important elevator heights that are pre-programmed into this class.
  enum Level {
    Bottom,
    L1,
    L2,
    L3,
    L4,
    Source
  }

  public Elevator() {
    configMotor(elevatorMotor1, false, 120.0); // Configures the motor with counterclockwise rotation positive and 25A current limit.
    configMotor(elevatorMotor2, true, 120.0); // Configures the motor with clockwise rotation positive and 25A current limit.
    elevatorMotor1.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorMotor2.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorMotor2.setControl(new Follower(9, true)); // Sets the slave motor to follow the master motor exactly.
  }

  // Controls the velocity of the elevator. 1.0 is full speed up, -1.0 is full speed down, 0.0 is stopped.
  public void manual(double speed) { 
    if (topLimitSwitch.get() && speed > 0.0) {  //If both limitSwich 2 is pressed and the speed is less than -1.0, set the speed to 0.
    elevatorMotor1.setControl(new DutyCycleOut(0.0));
    } else if (bottomLimitSwitch.get() && speed < 0.0) { // If both limitSwich 2 is pressed and the speed is less than -1.0, set the speed to 0.
    elevatorMotor1.setControl(new DutyCycleOut(0.0));
    } else {
    elevatorMotor1.setControl(new DutyCycleOut(speed));
    } 
  } 

  // Sets the elevator to a pre-programmed position. 
  public void setLevel(Level desiredLevel) {
    switch(desiredLevel) {
      case Bottom:
        setPosition(0.02);
      break;

      case L1:
        setPosition(0.23);
      break;

      case L2:
        setPosition(0.45); //0.45
      break;

      case L3:
        setPosition(0.87); //0.85
      break;
      
      case L4:
        setPosition(1.81); //1.79
      break;

      case Source:
        setPosition(0.11); //0.90
      break;
    }
  }

  // Checks if the motor is at the target position.
  public boolean atSetpoint() {
    return Math.abs(elevatorMotor1.getPosition().getValueAsDouble() - setpoint) < 0.1; // Checks if the motor is at the target position.
  }

  // Returns the current position of the elevator in meters.
  public double getPosition() {
    return (getMasterPosition() + getSlavePosition()) / 2.0;
  }

  // Returns true if the top limit switch is pressed.
  public boolean getTopLimitSwitch() {
    return !topLimitSwitch.get();
  }

  // Returns true if the bottom limit switch is pressed.
  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }

  // Updates the SmartDashboard with information about the elevator.
  public void updateDash() {
    SmartDashboard.putBoolean("Elevator Top Limit Switch", getTopLimitSwitch());
    SmartDashboard.putBoolean("Elevator Bottom Limit Switch", getBottomLimitSwitch());
    SmartDashboard.putNumber("Elevator Master Position", getMasterPosition());
    SmartDashboard.putNumber("Elevator Slave Position", getSlavePosition());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putBoolean("Elevator AtSetpoint", atSetpoint());
    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
  }

  // Sets the position of the elevator motor in meters.
  private void setPosition(double positionMeters) {
    if (positionMeters > highLimit) { // If the position is greater than the high limit, set the position to the high limit.
      positionMeters = highLimit;
    }
    if (positionMeters < lowLimit) { // If the position is less than the low limit, set the position to the low limit.
      positionMeters = lowLimit;
    }

    setpoint = (gearRatio * positionMeters) / (sprocketCircumference * elevatorRatio * correctionFactor);
    elevatorMotor1.setControl(new MotionMagicTorqueCurrentFOC(setpoint)); 
  }
  
  // Returns the position of the master elevator motor in meters.
  private double getMasterPosition() {
    return (elevatorMotor1.getPosition().getValueAsDouble() * sprocketCircumference * elevatorRatio * correctionFactor) / gearRatio;
  }

  // Returns the position of the slave elevator motor in meters.
  private double getSlavePosition() {
    return (elevatorMotor2.getPosition().getValueAsDouble() * sprocketCircumference * elevatorRatio * correctionFactor) / gearRatio;
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