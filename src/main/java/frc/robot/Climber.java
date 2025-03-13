package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
  private final TalonFX climbMasterMotor = new TalonFX(11, "canivore"); // Initializes the motor with CAN ID of 11 connected to the canivore. 
  private final TalonFX climbSlaveMotor = new TalonFX(16, "canivore"); // Initializes the motor with CAN ID of 16 connected to the canivore. 
  private final DutyCycleOut climbMasterMotorDutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(true); // Communicates duty cycle control requests to the climb motor.
  private final StatusSignal<Angle> climbMasterMotorPosition; // Stores the position of the climb motor.
  private final Servo latch = new Servo(0); // Initializes the servo motor connected to PWM port 0 on the RoboRIO.
  private final double lowLimit = 0.0; // The lowest point in the climbers range of motion in motor rotations.
  private final double highLimit = 122.0; // The highest point in the climbers range of motion in motor rotations.
  private boolean isLatched = false; // Stores whether the latch is engaged. Returns true if the climber is latched and locked into place.

  public Climber() {
    configMotor(climbMasterMotor, false); // Configures the motor with counterclockwise rotation positive and 120A current limit.
    configMotor(climbSlaveMotor, true); // Configures the motor with clockwise rotation positive and 120A current limit.
    climbMasterMotor.setPosition(0.0, 0.03); // Sets the position of the motor to 0 on startup.
    climbMasterMotorPosition = climbMasterMotor.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, climbMasterMotorPosition);
    ParentDevice.optimizeBusUtilizationForAll(climbMasterMotor, climbSlaveMotor);
    climbSlaveMotor.setControl(new Follower(climbMasterMotor.getDeviceID(), true));
    openLatch();
  }

  // Controls the velocity of the climber. 1.0 is full speed up, -1.0 is full speed down, 0.0 is stopped.
  public void setSpeed(double speed) {
    if (getPosition() < lowLimit && speed < 0.0)  speed = 0.0; // Turns the motor off if the climber is at its bottom limit and a down command is given.
    if (getPosition() > highLimit && speed > 0.0) speed = 0.0; // Turns the motor off if the climber is at its top limit and an up command is given.
    if (speed > 0.0 && isLatched()) speed = 0.0; // Turns the motor off if the climber is latched and an up command is given.
    climbMasterMotor.setControl(climbMasterMotorDutyCycleRequest.withOutput(speed)); // Sets the speed of the motor according to the command given.
  }

  // Directly controls the velocity of the climber without paying attention to limits. Used to reset the climber to its starting position.
  public void setSpeedBypassLimits(double speed) {
    climbMasterMotor.setControl(climbMasterMotorDutyCycleRequest.withOutput(speed)); // Sets the speed of the motor according to the command given.
  }

  // Opens the latch, allowing the climber to move freely.
  public void openLatch() {
    latch.set(1.0);
    isLatched = false;
  }

  // Closes the latch, locking the climber into place.
  public void closeLatch() {
    latch.set(0.2);
    isLatched = true;
  }

  // Returns true if the climber is latched and locked into place.
  public boolean isLatched() {
    return isLatched;
  }

  // Returns the current position of the climber in motor rotations. 
  public double getPosition() {
    return climbMasterMotorPosition.refresh().getValueAsDouble();
  }

  // Updates the SmartDashboard with information about the climber.
  public void updateDash() {
    //SmartDashboard.putBoolean("Climber isLatched", isLatched());
    //SmartDashboard.putNumber("Climber getPosition", getPosition());
  }

  private void configMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    
    motor.getConfigurator().apply(motorConfigs, 0.03);
  }
}