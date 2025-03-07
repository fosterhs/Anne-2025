package frc.robot;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AlgaeYeeter.ArmPosition;
import frc.robot.Elevator.Level;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.
  private final XboxController operator = new XboxController(1); // Initializes the operator controller.

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngAccTeleop / Drivetrain.maxAngVelTeleop);

  private double speedScaleFactor = 1.0; // Scales the speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Elevator elevator = new Elevator(); // Contains the elevator motor and limit switches.
  private final CoralSpitter coralSpitter = new CoralSpitter(); // Contains the coral ejector motor and coral sensor. 
  private final Climber climber = new Climber(); // Contains the climber motor.
  private final AlgaeYeeter algaeYeeter = new AlgaeYeeter(); // Contains the algae sensor, algae yeeter arm motor, and algae yeeter intake motors.
  private final CANdle leftCandle = new CANdle(0, "canivore"); // Initializes the lights on the left side of the robot.
  private final CANdle rightCandle = new CANdle(1, "canivore"); // Initializes the lights on the right side of the robot.
  private boolean candleStrobeState = true; // Stores whether the candles are on or off. Used to produce a stobe effect.
  private int period = 0; // Keeps track of how many periods have elapsed since the begining of the program.
  
  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "1-Piece Coral (Branch 2)";
  private static final String auto2 = "2-Piece Coral (Branch 2)";
  private static final String auto3 = "auto3";
  private String autoSelected;
  private int autoStage = 1;
  private enum scoreMode {Branch, L1, Algae};
  private scoreMode currScoreMode = scoreMode.Branch;

  // Auto Aim Variables
  private final double reefX = 176.75*0.0254; // The x-coordinate of the center of the reef in meters.
  private final double reefY = Drivetrain.fieldWidth/2.0; // The y-coordinate of the center of the reef in meters.
  private double[] scoringPositionsX = new double[30]; // Contains all scoring positions of the robot in the x-direction.
  private double[] scoringPositionsY = new double[30]; // Contains all scoring positions of the robot in the y-direction.
  private double[] scoringHeadings = new double[30]; // Contains all scoring headings of the robot.
  private double[] scoreDistances = new double[30]; // Stores the distance to each scoring location from the currenly position of the robot. Updated when scoreCalc() is called.
  private int nearestScoreIndex = 0; // Array index corresponding to the closest scoring location to the current position of the robot. Updated when scoreCalc() is called.

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    autoChooser.addOption(auto3, auto3);
    SmartDashboard.putData("Autos", autoChooser);

    SmartDashboard.putString("currScoreMode", "Branch");
    swerve.loadPath("Test", 0.0, 0.0, 0.0, 0.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    elevator.updateDash();
    coralSpitter.updateDash();
    algaeYeeter.updateDash();
    climber.updateDash();
    updateDash();

    // Causes the candleStrobeState variable to oscillate between true and false every 5 periods (0.1 seconds).
    period++;
    if (period % 5 == 0) candleStrobeState = !candleStrobeState;
  
    if (swerve.getAccurateCalibrationTimer() < 1.0 && candleStrobeState) { // Strobes the lights for 1 second after an accurate vision calibration is made.
      leftCandle.setLEDs(0, 0, 0, 0, 0, 8);
      rightCandle.setLEDs(0, 0, 0, 0, 0, 8);
    } else if (currScoreMode == scoreMode.Branch) { // White color for Branch mode.
      leftCandle.setLEDs(255, 255, 255, 0, 0, 8);
      rightCandle.setLEDs(255, 255, 255, 0, 0, 8);
    } else if (currScoreMode == scoreMode.L1) { // Purple color for L1 mode.
      leftCandle.setLEDs(255, 0, 255, 0, 0, 8);
      rightCandle.setLEDs(255, 0, 255, 0, 0, 8);
    } else if (currScoreMode == scoreMode.Algae) { // Green color for Algae Mode.
      leftCandle.setLEDs(0, 255, 0, 0, 0, 8);
      rightCandle.setLEDs(0, 255, 0, 0, 0, 8);
    }
    
  }

  public void autonomousInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    coralSpitter.init(); // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
    algaeYeeter.init(); // Should be called in autoInit() and teleopInit(). Required for the algaeYeeter to function correctly.
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
        swerve.resetDriveController(scoringHeadings[2]); // Prepares the robot to drive to the reef.
      break;

      case auto2:
        // AutoInit 2 code goes here.
        swerve.resetDriveController(scoringHeadings[2]); // Prepares the robot to drive to the reef.
      break;

      case auto3:
        // AutoInit 3 code goes here.
        swerve.resetPathController(0); // Prepares the robot to drive to the reef.
      break;
    }
  }

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(); // Updates the Limelights with the robot heading (for MegaTag2).
    coralSpitter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
    algaeYeeter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the algaeYeeter to function correctly.
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[2],scoringPositionsY[2],scoringHeadings[2]); // This moves the robot to the reef.
            elevator.setLevel(Level.L2); // This moves the elevator to the second level.
            if (swerve.atDriveGoal() && elevator.atSetpoint()) {
              autoStage = 2;
            }
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
            coralSpitter.spit(); // Spits the coral.
          break;
        }
      break;

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
            swerve.driveTo(scoringPositionsX[2],scoringPositionsY[2],scoringHeadings[2]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 2;
            }
          break; 

          case 2:
            // Auto 2, Stage 2 code goes here.
            elevator.setLevel(Level.L2); // This moves the elevator to the second level.       
            if (elevator.atSetpoint()) {
              autoStage = 3;
            }
          break;

          case 3:
            // Auto 2, Stage 3 code goes here.
            coralSpitter.spit();
            if (!coralSpitter.isSpitting()) {
              autoStage = 4;
            }
          break;

          case 4:
            // Auto 2, Stage 4 code goes here.
            elevator.setLevel(Level.bottom); // This moves the elevator to the Bottom level.]
            if (elevator.atSetpoint()) {
              autoStage = 5;
            }
          break;

          case 5:
            // Auto 2, Stage 5 code goes here.
            swerve.driveTo(scoringPositionsX[0],scoringPositionsY[0],scoringHeadings[0]); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 6;
            }
          break;

          case 6:
            // Auto 2, Stage 6 code goes here.
            swerve.driveTo(3.194,1.437,60.00 ); // This moves the robot to the source .
            if (swerve.atDriveGoal()) {
              autoStage = 7;
            }
          break;

          case 7:
          // Auto 2, Stage 7 code goes here.
           if(coralSpitter.coralDetected()){   // This checks if the coral is detected.
            autoStage = 8;                     
           }  
          break;

          case 8:
            // Auto 2, Stage 8 code goes here.
            swerve.driveTo(4.031, 3.005, 60.00); // This moves the robot to the reef.
            if (swerve.atDriveGoal()) {
              autoStage = 9;
            }
          break;

          case 9:
            // Auto 2, Stage 9 code goes here.
            elevator.setLevel(Level.L2); // This moves the elevator to the second level.       
            if (elevator.atSetpoint()) {
              autoStage = 10;
            }
          break;

          case 10:
            //Auto 2, Stage 10 code goes here.
            coralSpitter.spit();
            if (!coralSpitter.isSpitting()) {   // This checks if the coral is detected.
                autoStage = 11;
            }
          break;

          case 11:
            // Auto 2, Stage 10 code goes here.
            elevator.setLevel(Level.bottom); // This moves the elevator to the Bottom level.
          break;
        }   
      break;

    case auto3:
      switch (autoStage) {
        case 1:
          swerve.driveTo(5.965, 4.055, 180); // This moves the robot to the reef
          if (swerve.atDriveGoal() ) {
            autoStage = 2;
          }
        break;

        case 2:
          swerve.followPath(0);
          if (swerve.atPathEndpoint(0)) {
            autoStage = 3;
          }
        break;
      }
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    coralSpitter.init(); // Should be called in autoInit() and teleopInit(). Required for the coralSpitter to function correctly.
    algaeYeeter.init(); // Should be called in autoInit() and teleopInit(). Required for the algaeYeeter to function correctly.
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    // Sets both controllers to rumble for 0.7 seconds if a coral or algae has just been intaked or exhuasted.
    if ((coralSpitter.getExhaustTimer() > 0.1 && coralSpitter.getExhaustTimer() < 0.8) || (coralSpitter.getIntakeTimer() > 0.1 && coralSpitter.getIntakeTimer() < 0.8)
      || (algaeYeeter.getExhaustTimer() > 0.1 && algaeYeeter.getExhaustTimer() < 0.8) || (algaeYeeter.getIntakeTimer() > 0.1 && algaeYeeter.getIntakeTimer() < 0.8)) {
      operator.setRumble(RumbleType.kBothRumble, 1.0);
      driver.setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      operator.setRumble(RumbleType.kBothRumble, 0.0);
      driver.setRumble(RumbleType.kBothRumble, 0.0);
    }

    // The left center button (button 7) cycles through the 3 scoring modes of the robot.
    if (operator.getRawButtonPressed(7) && currScoreMode == scoreMode.Branch) {
      currScoreMode = scoreMode.L1;
      SmartDashboard.putString("currScoreMode", "L1");
    }
    if (operator.getRawButtonPressed(7) && currScoreMode == scoreMode.L1) {
      currScoreMode = scoreMode.Algae;
      SmartDashboard.putString("currScoreMode", "Algae");
    }
    if (operator.getRawButtonPressed(7) && currScoreMode == scoreMode.Algae) {
      currScoreMode = scoreMode.Branch;
      SmartDashboard.putString("currScoreMode", "Branch");
    }
    
    coralSpitter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the coralSpitter to function correctly.
    algaeYeeter.periodic(); // Should be called in autoPeroidic() and teleopPeriodic(). Required for the algaeYeeter to function correctly.

    if (driver.getRawButtonPressed(4)) speedScaleFactor = 1.0; // Y Button sets the drivetrain in full speed mode.
    if (driver.getRawButtonPressed(2)) speedScaleFactor = 0.6; // B button sets the drivetrain in medium speed mode.
    if (driver.getRawButtonPressed(1)) speedScaleFactor = 0.15; // A button sets the drivetrain in low speed mode.
    
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*speedScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (driver.getRawButton(3)) { // X button
      swerveLock = true; // Pressing the X-button causes the swerve modules to lock (for defense).
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else if (driver.getRawButtonPressed(6)) { // Right bumper button
      calcNearestScoringPose(); // Calculates the closest scoring position.
      System.out.println(nearestScoreIndex);
      swerve.resetDriveController(scoringHeadings[nearestScoreIndex]); // Prepares the robot to drive to the closest scoring position.
    } else if (driver.getRawButton(6)) { // Right bumper button
      swerve.driveTo(scoringPositionsX[nearestScoreIndex], scoringPositionsY[nearestScoreIndex], scoringHeadings[nearestScoreIndex]); // Drives to the closest scoring position.
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    if (driver.getRawButton(7)) { // Left center button
      for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
        swerve.addCalibrationEstimate(limelightIndex); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
      }
    }
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(); // Updates the position of the robot on the field based on previous calculations.  

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Right center button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
    
    // Controls the level of the elevator.
    if (operator.getRawButtonPressed(1)) elevator.setLevel(Level.L1); // A button
    if (operator.getRawButtonPressed(2)) elevator.setLevel(Level.L2); // B button
    if (operator.getRawButtonPressed(3)) elevator.setLevel(Level.L3); // X button
    if (operator.getRawButtonPressed(4)) elevator.setLevel(Level.L4); // Y button 
    if (operator.getLeftTriggerAxis() > 0.25) elevator.setLevel(Level.lowAlgae); // Left Trigger
    if (operator.getRightTriggerAxis() > 0.25) elevator.setLevel(Level.highAlgae); // Right Trigger
    if (algaeYeeter.getArmPosition() == ArmPosition.stow) {
      if (operator.getRawButtonPressed(5)) elevator.setLevel(Level.bottom); // Left bumper button
    }  

    // Controls the spitter
    if (operator.getRawButtonPressed(6)) coralSpitter.spit(); // Right bumper button
   
    // Controls the climber
    climber.setSpeed(MathUtil.applyDeadband(-operator.getLeftY(), 0.1)); // Left stick Y
    if (operator.getRawButtonPressed(8)) { // Right center button
      if (climber.isLatched()) {
        climber.openLatch();
      } else {
        climber.closeLatch();
      }
    }

    // Controls the algae yeeter.
    if (elevator.getLevel() != Level.bottom) {
      if (operator.getPOV() == 180) algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.algae); // D pad down
      if (operator.getPOV() == 90) algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.barge); // D pad left
    }
    if (operator.getPOV() == 0) algaeYeeter.setArmPosition(AlgaeYeeter.ArmPosition.stow); // D pad up
    if (operator.getPOV() == 270) algaeYeeter.yeet(); // D pad right
  }
  
  public void disabledInit() {    
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addCalibrationEstimate(limelightIndex); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    }
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Updates nearestScoreIndex to reflect the closest scoring location to the robot.
  public void calcNearestScoringPose() {
    // Calculates the distance to each scoring location using the distance formula.
    for (int i = 0 ; i < scoringPositionsX.length; i++) {
      scoreDistances[i] = Math.sqrt(Math.pow(scoringPositionsY[i] - swerve.getYPos(), 2) + Math.pow(scoringPositionsX[i] - swerve.getXPos(), 2));
    }

    double shortestDistance = scoreDistances[scoreDistances.length - 1]; // Stores the value of the shortest distance in the scoreDistances[] array.
    nearestScoreIndex = scoreDistances.length - 1; // Stores the index of the shortest distance in the scoreDistances[] array.

    // Finds the shortest distance in the scoreDistances[] array and updates nearestScoreIndex
    for (int i = 0; i < scoreDistances.length - 1; i++) {
      if ((currScoreMode == scoreMode.Branch) && (i <= 11 || i >= 24)) {
        if (scoreDistances[i] < shortestDistance ) {
          shortestDistance = scoreDistances[i];
          nearestScoreIndex = i;
        }
      } 
      if ((currScoreMode == scoreMode.L1) && ((i >= 12 && i <= 17) || i >= 24)) {
        if (scoreDistances[i] < shortestDistance ) {
          shortestDistance = scoreDistances[i];
          nearestScoreIndex = i;
        }
      } 
      if ((currScoreMode == scoreMode.Algae) && (i >= 18)) {
        if (scoreDistances[i] < shortestDistance ) {
          shortestDistance = scoreDistances[i];
          nearestScoreIndex = i;
        }
      } 
    }
  }

  // Calculates all of the scoring locations on the field.
  public void calcScoringPoses() {
    scoringPositionsX[0] = 3.822; // X-coordinates of the coral scoring locations in meters. Based on AprilTag 8, using the scoring position nearest to AprilTag 7 on the Red alliance.
    scoringPositionsY[0] = 2.924; // Y-coordinates of the coral scoring locations in meters. Based on AprilTag 8, using the scoring position nearest to AprilTag 7 on the Red alliance.
    scoringHeadings[0] = 60.0; // Heading of the robot at each coral scoring location in degrees. Based on AprilTag 8, using the scoring position nearest to AprilTag 7 on the Red alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 0th index by 60 degrees.
    for (int index = 1; index <= 5; index++) {
      scoringPositionsX[index] = (scoringPositionsX[0] - reefX)*Math.cos(Math.toRadians(index*60.0)) - (scoringPositionsY[0] - reefY)*Math.sin(Math.toRadians(index*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[0] - reefX)*Math.sin(Math.toRadians(index*60.0)) + (scoringPositionsY[0] - reefY)*Math.cos(Math.toRadians(index*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[0] + index*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }
    
    // Calculates the scoring position of the other scoring position on the same face of the reef by using an x-offset and a y-offset.
    scoringPositionsX[6] = scoringPositionsX[0] + 12.94*0.0254*Math.sin(Math.toRadians(60.0));
    scoringPositionsY[6] = scoringPositionsY[0] - 12.94*0.0254*Math.cos(Math.toRadians(60.0));
    scoringHeadings[6] = scoringHeadings[0];

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 6th index by 60 degrees.
    for (int index = 7; index <= 11; index++) {
      scoringPositionsX[index] = (scoringPositionsX[6] - reefX)*Math.cos(Math.toRadians((index-6)*60.0)) - (scoringPositionsY[6] - reefY)*Math.sin(Math.toRadians((index-6)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[6] - reefX)*Math.sin(Math.toRadians((index-6)*60.0)) + (scoringPositionsY[6] - reefY)*Math.cos(Math.toRadians((index-6)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[6] + (index-6)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }

    // L1 scoring position based on the April Tag 8 face of the reef on the Red Alliance.
    scoringPositionsX[12] = 3.822;
    scoringPositionsY[12] = 2.924;
    scoringHeadings[12] = -30.0;

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 12th index by 60 degrees.
    for (int index = 13; index <= 17; index++) {
      scoringPositionsX[index] = (scoringPositionsX[12] - reefX)*Math.cos(Math.toRadians((index-12)*60.0)) - (scoringPositionsY[12] - reefY)*Math.sin(Math.toRadians((index-12)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[12] - reefX)*Math.sin(Math.toRadians((index-12)*60.0)) + (scoringPositionsY[12] - reefY)*Math.cos(Math.toRadians((index-12)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[12] + (index-12)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }

    // Algae scoring position based on the April Tag 8 face of the reef on the Red Alliance.
    scoringPositionsX[18] = 3.822;
    scoringPositionsY[18] = 2.924;
    scoringHeadings[18] = 150.0;

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 18th index by 60 degrees.
    for (int index = 19; index <= 23; index++) {
      scoringPositionsX[index] = (scoringPositionsX[18] - reefX)*Math.cos(Math.toRadians((index-18)*60.0)) - (scoringPositionsY[18] - reefY)*Math.sin(Math.toRadians((index-18)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[18] - reefX)*Math.sin(Math.toRadians((index-18)*60.0)) + (scoringPositionsY[18] - reefY)*Math.cos(Math.toRadians((index-18)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[18] + (index-18)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }
    
    // These 4 scoring locations correspond to the source. There are 2 scoring locations at each of the 2 sources, for a total of 4. 
    scoringPositionsX[24] = 0.601; // X-position of the first scoring location at the source nearest the origin in meters.
    scoringPositionsY[24] = 1.343; // Y-position of the first scoring location at the source nearest the origin in meters.
    scoringHeadings[24] = 144.0; // Heading of the first scoring location at the source nearest the origin. The source makes 54 and 36 degree angles with the coordinate axes.

    scoringPositionsX[25] = 1.608; // X-position of the second scoring location at the source nearest the origin in meters.
    scoringPositionsY[25] = 0.592; // Y-position of the second scoring location at the source nearest the origin in meters.
    scoringHeadings[25] = scoringHeadings[12]; // This is automatically calculated. Does not need to be edited.

    scoringPositionsX[26] = scoringPositionsX[12]; // This is automatically calculated. Does not need to be edited.
    scoringPositionsY[26] = Drivetrain.fieldWidth - scoringPositionsY[12]; // This is automatically calculated. Does not need to be edited.
    scoringHeadings[26] = 36.0; // Heading of the first scoring location at the source furthest from the origin.

    scoringPositionsX[27] = scoringPositionsX[13]; // This is automatically calculated. Does not need to be edited.
    scoringPositionsY[27] = Drivetrain.fieldWidth - scoringPositionsY[13]; // This is automatically calculated. Does not need to be edited.
    scoringHeadings[27] = scoringHeadings[14]; // This is automatically calculated. Does not need to be edited.

    // The scoring location of the processor.
    scoringPositionsX[28] = 11.561; // X-position of the processor scoring location in meters.
    scoringPositionsY[28] = 7.609; // Y-position of the processor scoring location in meters.
    scoringHeadings[28] = 90.0; // Heading of the processor scoring location.

    // The scoring location of the barge.
    scoringPositionsX[29] = 7.500; // X-position of the barge scoring location in meters.
    scoringPositionsY[29] = 6.000; // Y-position of the barge scoring location in meters.
    scoringHeadings[29] = -90.0; // Heading of the barge scoring location.
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  public void runAll() { 
    swerve.resetDriveController(0.0);
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0, false);
    swerve.driveTo(1.0, -2.0, -75.0);
    swerve.resetPathController(0);
    swerve.followPath(0);
    if (swerve.limelights.length > 0) swerve.addCalibrationEstimate(0);
    swerve.pushCalibration();
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.updateVisionHeading();
    if (swerve.limelights.length > 0) swerve.addVisionEstimate(0);
    swerve.updateOdometry();
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVel: " + swerve.getAngVel());
    System.out.println("swerve getCalibrationTimer: " + swerve.getCalibrationTimer());
    System.out.println("swerve getFusedAng: " + swerve.getFusedAng());
    System.out.println("swerve getGyroAng: " + swerve.getGyroAng());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getPathAngleError: " + swerve.getPathAngleError());
    System.out.println("swerve getPathPosError: " + swerve.getPathPosError());
    System.out.println("swerve getXPos: " + swerve.getXPos());
    System.out.println("swerve getXVel: " + swerve.getXVel());
    System.out.println("swerve getYPos: " + swerve.getYPos());
    System.out.println("swerve getYVel: " + swerve.getYVel());
    System.out.println("swerve isBlueAlliance: " + swerve.isBlueAlliance());
    System.out.println("swerve isRedAlliance: " + swerve.isRedAlliance());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getAngleDist: " + swerve.getAngleDistance(30.0, -120.0));
    swerve.updateDash();

    coralSpitter.init();
    coralSpitter.periodic();
    coralSpitter.spit();
    System.out.println("spitter coralDetected: " + coralSpitter.coralDetected());
    System.out.println("spitter getIntakeSensor: " + coralSpitter.getIntakeSensor());
    System.out.println("spitter getExhaustSensor: " + coralSpitter.getExhaustSensor());
    System.out.println("spitter isSpitting: " + coralSpitter.isSpitting());
    System.out.println("spitter intakeTimer: " + coralSpitter.getIntakeTimer());
    System.out.println("spitter exhaustTimer: " + coralSpitter.getExhaustTimer());
    coralSpitter.updateDash();

    elevator.setLevel(Level.bottom);
    if (elevator.getLevel() == Level.bottom) System.out.println("elevator at bottom");
    System.out.println("elevator atSetpoint: " + elevator.atSetpoint());
    System.out.println("elevator getPosition: " + elevator.getPosition());
    elevator.updateDash();

    climber.setSpeed(0.0);
    climber.closeLatch();
    climber.openLatch();
    System.out.println("climber position: " + climber.getPosition());
    System.out.println("climber isLatched: " + climber.isLatched());

    algaeYeeter.init();
    algaeYeeter.periodic();
    algaeYeeter.setArmPosition(ArmPosition.stow);
    algaeYeeter.yeet();
    if (algaeYeeter.getArmPosition() == ArmPosition.stow) System.out.println("algae yeeter stowed");
    System.out.println("algae yeeter getArmAngle: " + algaeYeeter.getArmAngle());
    System.out.println("algae yeeter isYeeting: " + algaeYeeter.isYeeting());
    System.out.println("algae yeeter algaeDetected: " + algaeYeeter.algaeDetected());
    System.out.println("algae yeeter armAtSetpoint: " + algaeYeeter.armAtSetpoint());
    System.out.println("algae yeeter intakeTimer: " + algaeYeeter.getIntakeTimer());
    System.out.println("algae yeeter exhaustTimer: " + algaeYeeter.getExhaustTimer());

    updateDash();
    calcScoringPoses();
    calcNearestScoringPose();
  }
}