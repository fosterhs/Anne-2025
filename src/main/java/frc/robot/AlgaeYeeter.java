package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeYeeter {
  private final TalonFX armMotor = new TalonFX(13, "rio"); // Initializes the motor with CAN ID of 13 connected to the roboRIO. 
  private final TalonFX intakeMasterMotor = new TalonFX(15, "rio");  // Initializes the motor with CAN ID of 14 connected to the roboRIO. 
  private final TalonFX intakeSlaveMotor = new TalonFX(14, "rio");  // Initializes the motor with CAN ID of 15 connected to the roboRIO. 
  private final VoltageOut intakeMasterMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Communicates voltage requests to the master intake motor.
  private final TorqueCurrentFOC intakeMasterMotorTorqueRequest = new TorqueCurrentFOC(0.0); // Comunicates current requests to the master intake motor.
  private final VoltageOut intakeSlaveMotorVoltageRequest = new VoltageOut(0.0).withEnableFOC(true); // Communicates voltage requests to the slave intake motor.
  private final TorqueCurrentFOC intakeSlaveMotorTorqueRequest = new TorqueCurrentFOC(0.0); // Comunicates current requests to the slave intake motor.
  private final MotionMagicTorqueCurrentFOC armMotorPositionRequest = new MotionMagicTorqueCurrentFOC(0.0); // Communicates motion magic torque current FOC position requests to the arm motor.
  private final StatusSignal<Angle> armMotorPosition; // Stores the position of the arm motor.
  private final DigitalInput algaeSensor = new DigitalInput(2); // Initializes the sensor connected to DIO port 2 on the RoboRIO.
  private final Timer algaeIntakeTimer = new Timer(); // Keeps track of how long has passed since an algae was first detected.
  private final Timer algaeExhaustTimer = new Timer(); // Keeps track of how long has passed since an algae has stopped being detected.
  private final double exhaustDelay = 1.0; // How long the wheels will continue running for after an algae is no longer detected in seconds.
  private final double intakeDelay = 1.0; // How long the wheels will wait before stopping to spin after an algae is detected.
  public enum ArmPosition {algae, barge, stow} // A list containing important arm positions that are pre-programmed.
  private final double highLimit = 0.25; // The high limit of the arm motor in mechanism rotations.
  private final double lowLimit = 0.0; // The low limit of the arm motor in mechanism rotations.
  private final double posTol = 0.01; // How much error is acceptable between the setpoint and the current position of the elevator in mechanism rotations.
  private double setpoint = 0.25; // The position that the arm motor is trying to reach in mechanism rotations.
  private ArmPosition currPosition = ArmPosition.stow; // Stores the last commanded position of the arm.
  private boolean isYeeting = false; // Returns true if the yeeter is in the process of launching an algae. 

  public AlgaeYeeter() {
    configArmMotor(armMotor, false, 120.0); // Configures the motor with counterclockwise rotation positive and 120A current limit. 
    configIntakeMotor(intakeMasterMotor, true, 120.0); // Configures the motor with counterclockwise rotation positive and 120A current limit. 
    configIntakeMotor(intakeSlaveMotor, false, 120.0); // Configures the motor with clockwise rotation positive and 120A current limit. 
    armMotor.setPosition(0.25, 0.03); // Sets the position of the motor to 0 on startup.
    armMotorPosition = armMotor.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, armMotorPosition);
    ParentDevice.optimizeBusUtilizationForAll(armMotor, intakeMasterMotor, intakeSlaveMotor);
    algaeExhaustTimer.restart();
    algaeIntakeTimer.restart();
  }

  // Should be called in autoInit() and teleopInit(). Required for the algaeYeeter to function correctly.
  public void init() { 
    isYeeting = false;
  }

  // Should be called in autoPeroidic() and teleopPeriodic(). Required for the algaeYeeter to function correctly.
  public void periodic() {
    if (algaeDetected()) algaeExhaustTimer.restart();
    if (!algaeDetected()) algaeIntakeTimer.restart();
    if (algaeExhaustTimer.get() > exhaustDelay || currPosition == ArmPosition.stow) isYeeting = false;

    if (isYeeting) {
      intakeMasterMotor.setControl(intakeMasterMotorVoltageRequest.withOutput(-12.0)); // Sets the intake motors to exhaust at 12 volts.
      intakeSlaveMotor.setControl(intakeSlaveMotorVoltageRequest.withOutput(-12.0));
    } else if (currPosition == ArmPosition.stow) {
      intakeMasterMotor.setControl(intakeMasterMotorVoltageRequest.withOutput(0.0)); // Sets the intake motors off.
      intakeSlaveMotor.setControl(intakeSlaveMotorVoltageRequest.withOutput(0.0));
    } else if (algaeIntakeTimer.get() < intakeDelay) {
      intakeMasterMotor.setControl(intakeMasterMotorVoltageRequest.withOutput(2.0)); // Sets the intake motors to intake at 2 volts.
      intakeSlaveMotor.setControl(intakeSlaveMotorVoltageRequest.withOutput(2.0));
    } else {
      intakeMasterMotor.setControl(intakeMasterMotorTorqueRequest.withOutput(5.0)); // Sets the intake motors to hold the algae with 5A of current. 
      intakeSlaveMotor.setControl(intakeSlaveMotorTorqueRequest.withOutput(5.0));
    }
  }

  // Sets the arm to a pre-programmed position. 
  public void setArmPosition(ArmPosition desiredPosition) {
    switch(desiredPosition) {
      case algae:
        setArmMotorRotations(0.0);
        currPosition = ArmPosition.algae;
      break;

      case barge:
        setArmMotorRotations(0.17);
        currPosition = ArmPosition.barge;
      break;
      
      case stow:
        if (!algaeDetected()) {
          setArmMotorRotations(0.25);
          currPosition = ArmPosition.stow;
        }
      break;
    }
  }

  // Tells the algaeYeeter to begin the process of ejecting an algae. 
  public void yeet() { 
    isYeeting = algaeDetected();
  }

  // Returns the last requested position of the arm.
  public ArmPosition getArmPosition() {
    return currPosition;
  }

  // Returns true if the algaeYeeter is in the process of ejecting an algae.
  public boolean isYeeting() {
    return isYeeting;
  }

  // Returns true if an algae is detected by the sensor.
  public boolean algaeDetected() {
    return !algaeSensor.get();
  }

  // Checks if the arm motor is at the target position.
  public boolean armAtSetpoint() {
    return Math.abs(getArmAngle() - setpoint) < posTol; // Checks if the motor is at the target position.
  }

  // Returns the current position of the arm in mechanism rotations. 
  public double getArmAngle() {
    return armMotorPosition.refresh().getValueAsDouble();
  }

  // Returns the current value of the exhaust timer
  public double getExhaustTimer() {
    return algaeExhaustTimer.get();
  }

  // Returns the current value of the intake timer
  public double getIntakeTimer() {
    return algaeIntakeTimer.get();
  }

  // Updates the SmartDashboard with information about the algae yeeter.
  public void updateDash() {
    SmartDashboard.putBoolean("Algae Yeeter algaeDetected", algaeDetected());
    SmartDashboard.putNumber("Algae Yeeter getArmAngle", getArmAngle());
    SmartDashboard.putBoolean("Algae Yeeter armAtSetpoint", armAtSetpoint());
    SmartDashboard.putBoolean("Algae Yeeter isYeeting", isYeeting());
    SmartDashboard.putNumber("Algae Yeeter Intake Timer", getExhaustTimer());
    SmartDashboard.putNumber("Algae Yeeter Exhaust Timer", getIntakeTimer());
    SmartDashboard.putNumber("Algae Yeeter Setpoint", setpoint);
  }

  // Sets the arm to the desired position in mechanism rotations.
  private void setArmMotorRotations(double desiredRotations) {
    if (desiredRotations > highLimit) desiredRotations = highLimit; // If the position is greater than the high limit, set the position to the high limit.
    if (desiredRotations < lowLimit) desiredRotations = lowLimit; // If the position is less than the low limit, set the position to the low limit.
    armMotor.setControl(armMotorPositionRequest.withPosition(desiredRotations)); // Sets the position of the motor in shaft rotations.
    setpoint = desiredRotations;
  }

  private void configArmMotor(TalonFX motor, boolean invert, double currentLimit) {
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
  
    motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  
    // Current limit configuration.
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
  
    // MotionMagicTorqueFOC closed-loop control configuration.
    motorConfigs.Slot0.kP = 37.0*50.0; // Units: amperes per 1 rotation of error.
    motorConfigs.Slot0.kI = 0.0; // Units: amperes per 1 rotation * 1 second of error.
    motorConfigs.Slot0.kD = 0.84*50.0; // Units: amperes per 1 rotation / 1 second of error.
    motorConfigs.Slot0.kG = 5.0; // output to overcome gravity
    motorConfigs.Slot0.kS = 1.0; // Units: amperes.
    motorConfigs.MotionMagic.MotionMagicAcceleration = 20.0; // Units: rotations per second per second.
    motorConfigs.MotionMagic.MotionMagicCruiseVelocity = 2.0; // Units: rotations per second.
    motorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfigs.Feedback.SensorToMechanismRatio = 50.0;
  
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