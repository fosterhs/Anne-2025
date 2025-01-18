package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Elevator {
  public final TalonFX elevatorMotor1 = new TalonFX(9, "canivore");
  private final TalonFX elevatorMotor2 = new TalonFX(10, "canivore");
  private final DigitalInput limitSwich1 = new DigitalInput(0); // Initializes the sensor connected to DIO port 0 on the RoboRIO.
  private final DigitalInput limitSwich2 = new DigitalInput(1); // Initializes the sensor connected to DIO port 0 on the RoboRIO.

  public double setPoint = 0.0; // The position that the elevator motor is trying to reach in motor rotations.
  private double sprocketCircumference = Math.PI * 1.889 * 0.0254; // The diameter of the sprocket in meters.
  private double gearRatio = 12.0; // The gear ratio of the elevator motor.
  private double elevatorRatio = 2.0; // The gear ratio of the elevator motor.
  private double highLimit = 2.0; // The high limit of the elevator motor in meters.
  private double lowLimit = 0.0; // The low limit of the elevator motor in meters.

  public Elevator() {
    configMotor(elevatorMotor1, false, 15.0); // Configures the motor with counterclockwise rotation positive and 80A current limit.
    configMotor(elevatorMotor2, true, 15.0); // Configures the motor with counterclockwise rotation positive and 80A current limit.

    elevatorMotor1.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorMotor2.setPosition(0.0, 0.03); // Sets the position of the motor to 0.
    elevatorMotor2.setControl(new Follower(9, true));
  }
  

  // 1.0 is up, -1.0 is down 0.0 is stop
  public void manualElevator(Double speed) { 
   /*  if (limitSwich1.get() && speed <= 1.0) {  //If both limitSwich 2 is pressed and the speed is less than -1.0, set the speed to 0.
      elevatorMotor1.setControl(new DutyCycleOut(0.0));
      } else if (limitSwich2.get() && speed >= -1.0) { // If both limitSwich 2 is pressed and the speed is less than -1.0, set the speed to 0.
      elevatorMotor1.setControl(new DutyCycleOut(0.0));
      } else {
      elevatorMotor1.setControl(new DutyCycleOut(speed));
      } */
      elevatorMotor1.setControl(new DutyCycleOut(speed));
  } 
    
  // Sets the position of the elevator motor in meters.
  public void setElevatorPosition(double positionMeters) {
    double positionRotations =  (gearRatio * positionMeters) / (sprocketCircumference * elevatorRatio);
    elevatorMotor1.setControl(new MotionMagicTorqueCurrentFOC(positionRotations)); 
    setPoint = positionRotations;
    elevatorMotor1.setControl(new DutyCycleOut(positionRotations)); // Sets the position of the motor.

    if (positionMeters > highLimit) { // If the position is greater than the high limit, set the position to the high limit.
      positionMeters = highLimit;
    }
    if (positionMeters < lowLimit) { // If the position is less than the low limit, set the position to the low limit.
      positionMeters = lowLimit;
    }
  }

  // Checks if the motor is at the target position.
  public boolean isAtSetpoint() {
    return Math.abs(elevatorMotor1.getPosition().getValueAsDouble() - setPoint) < 0.1; // Checks if the motor is at the target position.
  }
  
  // Returns the position of the elevator motor in meters.
  public double getElevatorMasterPosition() {
    return (elevatorMotor1.getPosition().getValueAsDouble() * sprocketCircumference * elevatorRatio) / gearRatio;
  }

  public double getElevatorSlavePosition() {
    return (elevatorMotor2.getPosition().getValueAsDouble() * sprocketCircumference * elevatorRatio) / gearRatio;
  }

  // Returns whether the limit switch is pressed.
  public boolean isTouchingLimitSwitch() {
    return limitSwich1.get();
  }

  // Updates the SmartDashboard with information about the elevator.
  public void updateDash() {
    SmartDashboard.putBoolean("Elevator Touching LimitSwitch", isTouchingLimitSwitch());
    SmartDashboard.putNumber("Elevator Master Position", getElevatorMasterPosition());
    SmartDashboard.putNumber("Elevator Slave Position", getElevatorSlavePosition());
    SmartDashboard.putBoolean("Elevator AtSetpoint", isAtSetpoint());
    SmartDashboard.putNumber("Elevator Setpoint", setPoint);
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
    motorConfigs.Slot0.kP = 37.0; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 0.84; // Units: amperes per 1 rotation / 1 second of error.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 1000.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0; // Units: rotations per second.
  
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}
