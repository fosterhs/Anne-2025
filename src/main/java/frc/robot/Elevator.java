package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



class Elevator {
     //Variables for the elevator
    private final TalonFX elevatorMotor1 = new TalonFX(9, "canivore");
    private final DigitalInput limitSwich1 = new DigitalInput(0); // Initializes the sensor connected to DIO port 0 on the RoboRIO.


    private boolean isAtSetpoint = false;
    private double setPoint = 0.0;
   

    
   

    public Elevator() {
        configMotor(elevatorMotor1, false, 80.0); // Configures the motor with counterclockwise rotation positive and 80A current limit.

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
       // Sets the position of the elevator motor in meters.
      public void setElevatorPosition(double positionMeters) {
        double positionRotations = (positionMeters * 1.889 * 2.0 * Math.PI) / ( 12 * 0.0254);
        elevatorMotor1.setControl(new MotionMagicTorqueCurrentFOC(positionRotations)); 
        setPoint = positionRotations;
      }
      // Checks if the motor is at the target position.
      public boolean isAtSetpoint() {
        return Math.abs(elevatorMotor1.getPosition().getValueAsDouble() - setPoint) < 0.1; // Checks if the motor is at the target position.
      }
      // Returns the position of the elevator motor in meters.
      public double getElevatorPosition() {
        return (elevatorMotor1.getPosition().getValueAsDouble() * 12.0 * 0.0254) / (1.889 * 2.0 * Math.PI);
      }
      // Returns whether the limit switch is pressed.
      public boolean isTouchingLimitSwitch() {
        return limitSwich1.get();
      }

    public void updateDash() {
      SmartDashboard.putBoolean("Elevator Touching LimitSwitch", isTouchingLimitSwitch());
      SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
      SmartDashboard.putBoolean("Elevator AtSetpoint", isAtSetpoint());
      SmartDashboard.putNumber("Elevator Setpoint", setPoint);
    }
}
