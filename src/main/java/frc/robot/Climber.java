package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
  private final TalonFX climbMotor = new TalonFX(11, "canivore"); // Initializes the motor with CAN ID of 11 connected to the canivore. 
  private final Servo latch = new Servo(0); // Initializes the servo motor connected to PWM port 0 on the RoboRIO.
  private boolean isLatched = false; // Stores whether the latch is engaged. Returns true if the climber is latched and locked into place.

  public Climber() {
    configMotor(climbMotor, false, 120.0); // Configures the motor with counterclockwise rotation positive and 80A current limit. 
    openLatch();
  }

  // Controls the velocity of the climber. 1.0 is full speed up, -1.0 is full speed down, 0.0 is stopped.
  public void setSpeed(double speed) {
    climbMotor.setControl(new DutyCycleOut(speed)); // Sets the speed of the motor.
  }

  // Opens the latch, allowing the climber to move freely.
  public void openLatch() {
    latch.set(0.0);
    isLatched = false;
  }

  // Closes the latch, locking the climber into place.
  public void closeLatch() {
    latch.set(1.0);
    isLatched = true;
  }

  // Returns true if the climber is latched and locked into place.
  public boolean isLatched() {
    return isLatched;
  }

  // Updates the SmartDashboard with information about the climber.
  public void updateDash() {
    //SmartDashboard.putBoolean("Climber isLatched", isLatched());
  }

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