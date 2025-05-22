package frc.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TShirtLauncher {
    
    private final VictorSPX motor1VictorSPX = new VictorSPX(5); //Activates VictorSPS Moter 1
    private final VictorSPX motor2VictorSPX = new VictorSPX(2); //Activates VictorSPS Moter 2
    private final AnalogPotentiometer voltRead = new AnalogPotentiometer(0, 5, 0); // Potentiometer on channel 0, range 0-360 degree
    
    private double PSIRead = (49.8 * voltRead.get()) + -26.1;

    public TShirtLauncher() {
        // Constructor for the Luncher class
        motor1VictorSPX.configFactoryDefault(); 
        motor2VictorSPX.configFactoryDefault();
    }

    public void setSolenoid1(double PercentOutput) {
        // Set the speed of solenoid 1
        motor1VictorSPX.set(VictorSPXControlMode.PercentOutput, PercentOutput);
    }

    public void setSolenoid2(double PercentOutput) {
         // Set the speed of solenoid 2
        motor2VictorSPX.set(VictorSPXControlMode.PercentOutput, PercentOutput);
    }
    
    public double getSensor1() {
        return PSIRead;
    }

    // Updates the SmartDashboard with information about the Launcher.
    public void updateDash() {
        SmartDashboard.putNumber("PSI Value", getSensor1());
        SmartDashboard.putNumber("Volt Read", voltRead.get());
    }
}
