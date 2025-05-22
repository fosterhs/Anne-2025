package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController driver = new XboxController(0); // Initializes the driver controller.
  private final XboxController operator = new XboxController(1); // Initializes the operator controller.
  

  // Limits the acceleration of the drivetrain by smoothing controller inputs.
  private final SlewRateLimiter xAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter yAccLimiter = new SlewRateLimiter(Drivetrain.maxAccTeleop / Drivetrain.maxVelTeleop);
  private final SlewRateLimiter angAccLimiter = new SlewRateLimiter(Drivetrain.maxAngAccTeleop / Drivetrain.maxAngVelTeleop);

  private double speedScaleFactor = 0.65; // Scales the translational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private double rotationScaleFactor = 0.3; // Scales the rotational speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean boostMode = false; // Stores whether the robot is at 100% speed (boost mode), or at ~65% speed (normal mode).
  private boolean swerveLock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. 

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final TShirtLauncher launcher = new TShirtLauncher(); // Contains the launcher for the t-shirt cannon. This is not used in the 2024 game, but is included for completeness.

  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "WIP1"; 
  private String autoSelected;
  private int autoStage = 1;
  private Timer coralTimer = new Timer();

  // Auto Aim Variables
  private final double reefX = 176.75*0.0254; // The x-coordinate of the center of the reef in meters.
  private final double reefY = Drivetrain.fieldWidth/2.0; // The y-coordinate of the center of the reef in meters.
  private double[] scoringPositionsX = new double[30]; // Contains all scoring positions of the robot in the x-direction.
  private double[] scoringPositionsY = new double[30]; // Contains all scoring positions of the robot in the y-direction.
  private double[] scoringHeadings = new double[30]; // Contains all scoring headings of the robot.
  private double[] scoreDistances = new double[30]; // Stores the distance to each scoring location from the currenly position of the robot. Updated when scoreCalc() is called.
  private int nearestScoreIndex = 0; // Array index corresponding to the closest scoring location to the current position of the robot. Updated when scoreCalc() is called.
  private enum scoreMode {Branch, L1, Algae};
  private scoreMode currScoreMode = scoreMode.Branch;

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);

    SmartDashboard.putData("Autos", autoChooser);

    coralTimer.restart();
    swerve.loadPath("Test", 0.0, 0.0, 0.0, 0.0); // Loads a Path Planner generated path into the path follower code in the drivetrain.
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    updateDash();
    SmartDashboard.putNumber("launcher pot", launcher.getSensor1());
  }

  public void autonomousInit() {
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
      break;
    }
  }

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
          break;
        }
      break;   
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(true, swerve.getFusedAng()); // Updates the robot's position on the field.
  }

  public void teleopPeriodic() {
    if (operator.getRawButton(1)) {
      launcher.setSolenoid2(1); 
    } else {
      launcher.setSolenoid2(0);
    }

    if (operator.getRawButton(2) && operator.getRightTriggerAxis() > 0.1 && operator.getLeftTriggerAxis() > 0.1) {
      launcher.setSolenoid1(1); 
    } else {
      launcher.setSolenoid1(0);
    }

    
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    swerve.updateVisionHeading(false, 0.0); // Updates the Limelights with the robot heading (for MegaTag2).
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    if (driver.getRawButtonPressed(1)) boostMode = true; // A button sets boost mode. (100% speed up from default of 60%).
    if (driver.getRawButtonPressed(2)) boostMode = false; // B Button sets default mode (60% of full speed).

    
    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*rotationScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (driver.getRawButton(3)) { // X button
      swerveLock = true; // Pressing the X-button causes the swerve modules to lock (for defense).
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      swerveLock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    if (swerveLock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else if (driver.getRawButtonPressed(6)) { // Right bumper button
      calcNearestScoringPose(); // Calculates the closest scoring position.
      swerve.resetDriveController(scoringHeadings[nearestScoreIndex]); // Prepares the robot to drive to the closest scoring position.
    } else if (driver.getRawButton(6)) { // Right bumper button
      if (nearestScoreIndex < 18) {
        swerve.driveTo(scoringPositionsX[nearestScoreIndex], scoringPositionsY[nearestScoreIndex], scoringHeadings[nearestScoreIndex]); // Drives to the closest scoring position.
      } else {
        swerve.aimDrive(xVel, yVel, scoringHeadings[nearestScoreIndex], true);
      }
    } else {
      swerve.drive(xVel, yVel, angVel, true, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }



    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) {
      swerve.calcPriorityLimelightIndex();
      swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    }
    if (driver.getRawButton(7)) swerve.addCalibrationEstimate(swerve.getPriorityLimelightIndex(), false); // Left center button
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(false, 0.0); // Updates the position of the robot on the field based on previous calculations.  

    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Right center button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
  }
  
  public void disabledInit() { 
    swerve.calcPriorityLimelightIndex();
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    SmartDashboard.putBoolean("Boost Mode", boostMode);
    //SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    //SmartDashboard.putNumber("Auto Stage", autoStage);
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
    scoringPositionsX[0] = (144.0-26.0/2.0-3.375)*0.0254; // X-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the larger y-coordinate on the blue alliance.
    scoringPositionsY[0] = Drivetrain.fieldWidth/2.0 + 12.94*0.0254/2.0 - 10.25*0.0254; // Y-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the larger y-coordinate on the blue alliance.
    scoringHeadings[0] = 0.0; // Heading of the robot at the coral scoring location in degrees. Based on the scoring location closest to the alliance wall with the larger y-coordinate on the blue alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 0th index by 60 degrees.
    for (int index = 1; index <= 5; index++) {
      scoringPositionsX[index] = (scoringPositionsX[0] - reefX)*Math.cos(Math.toRadians(index*60.0)) - (scoringPositionsY[0] - reefY)*Math.sin(Math.toRadians(index*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[0] - reefX)*Math.sin(Math.toRadians(index*60.0)) + (scoringPositionsY[0] - reefY)*Math.cos(Math.toRadians(index*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[0] + index*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }
    
    // Calculates the scoring position of the other scoring position on the same face of the reef by using an x-offset and a y-offset.
    scoringPositionsX[6] = (144.0-26.0/2.0-3.375)*0.0254; // X-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringPositionsY[6] = Drivetrain.fieldWidth/2.0 - 12.94*0.0254/2.0 - 10.25*0.0254; // Y-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringHeadings[6] = 0.0; // Heading of the robot at the coral scoring location in degrees. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 6th index by 60 degrees.
    for (int index = 7; index <= 11; index++) {
      scoringPositionsX[index] = (scoringPositionsX[6] - reefX)*Math.cos(Math.toRadians((index-6)*60.0)) - (scoringPositionsY[6] - reefY)*Math.sin(Math.toRadians((index-6)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[6] - reefX)*Math.sin(Math.toRadians((index-6)*60.0)) + (scoringPositionsY[6] - reefY)*Math.cos(Math.toRadians((index-6)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[6] + (index-6)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }

    // L1 scoring position based on the April Tag 8 face of the reef on the Red Alliance.
    scoringPositionsX[12] = 3.166;
    scoringPositionsY[12] = 3.640;
    scoringHeadings[12] = -15.0;

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 12th index by 60 degrees.
    for (int index = 13; index <= 17; index++) {
      scoringPositionsX[index] = (scoringPositionsX[12] - reefX)*Math.cos(Math.toRadians((index-12)*60.0)) - (scoringPositionsY[12] - reefY)*Math.sin(Math.toRadians((index-12)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[12] - reefX)*Math.sin(Math.toRadians((index-12)*60.0)) + (scoringPositionsY[12] - reefY)*Math.cos(Math.toRadians((index-12)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[12] + (index-12)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }

    // Algae scoring position based on the April Tag 8 face of the reef on the Red Alliance.
    scoringPositionsX[18] = (144.0-33.5/2.0-3.375)*0.0254; // X-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringPositionsY[18] = Drivetrain.fieldWidth/2.0; // Y-coordinates of the coral scoring location in meters. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.
    scoringHeadings[18] = -90.0; // Heading of the robot at the coral scoring location in degrees. Based on the scoring location closest to the alliance wall with the smaller y-coordinate on the blue alliance.

    // Calculates the scoring positions of the remaining 5 faces of the reef by rotating the coordinates of the 18th index by 60 degrees.
    for (int index = 19; index <= 23; index++) {
      scoringPositionsX[index] = (scoringPositionsX[18] - reefX)*Math.cos(Math.toRadians((index-18)*60.0)) - (scoringPositionsY[18] - reefY)*Math.sin(Math.toRadians((index-18)*60.0)) + reefX;
      scoringPositionsY[index] = (scoringPositionsX[18] - reefX)*Math.sin(Math.toRadians((index-18)*60.0)) + (scoringPositionsY[18] - reefY)*Math.cos(Math.toRadians((index-18)*60.0)) + reefY;
      scoringHeadings[index] = scoringHeadings[18] + (index-18)*60.0;
      if (scoringHeadings[index] > 180.0) scoringHeadings[index] -= 360.0;
      if (scoringHeadings[index] < -180.0) scoringHeadings[index] += 360.0;
    }
    
    // These 4 scoring locations correspond to the source. There are 2 scoring locations at each of the 2 sources, for a total of 4. 
    scoringPositionsX[24] = 1.037; // X-position of the first scoring location at the source nearest the origin in meters.
    scoringPositionsY[24] = 0.885; // Y-position of the first scoring location at the source nearest the origin in meters.
    scoringHeadings[24] = 144.0; // Heading of the first scoring location at the source nearest the origin. The source makes 54 and 36 degree angles with the coordinate axes.

    scoringPositionsX[25] = 1.037; // X-position of the second scoring location at the source nearest the origin in meters.
    scoringPositionsY[25] = 0.885; // Y-position of the second scoring location at the source nearest the origin in meters.
    scoringHeadings[25] = scoringHeadings[24]; // This is automatically calculated. Does not need to be edited.

    scoringPositionsX[26] = scoringPositionsX[24]; // This is automatically calculated. Does not need to be edited.
    scoringPositionsY[26] = Drivetrain.fieldWidth - scoringPositionsY[24]; // This is automatically calculated. Does not need to be edited.
    scoringHeadings[26] = 36.0; // Heading of the first scoring location at the source furthest from the origin.

    scoringPositionsX[27] = scoringPositionsX[25]; // This is automatically calculated. Does not need to be edited.
    scoringPositionsY[27] = Drivetrain.fieldWidth - scoringPositionsY[25]; // This is automatically calculated. Does not need to be edited.
    scoringHeadings[27] = scoringHeadings[26]; // This is automatically calculated. Does not need to be edited.

    // The scoring location of the processor.
    scoringPositionsX[28] = 11.561; // X-position of the processor scoring location in meters.
    scoringPositionsY[28] = Drivetrain.fieldWidth - 33.5*0.0254/2; // Y-position of the processor scoring location in meters.
    scoringHeadings[28] = 0.0; // Heading of the processor scoring location.

    // The scoring location of the barge.
    scoringPositionsX[29] = 7.697; // X-position of the barge scoring location in meters.
    scoringPositionsY[29] = 4.910; // Y-position of the barge scoring location in meters.
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
    swerve.addCalibrationEstimate(0, false);
    swerve.pushCalibration(true, 180.0);
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.updateVisionHeading(true, 180.0);
    swerve.addVisionEstimate(0, true);
    swerve.updateOdometry();
    swerve.drive(0.01, 0.0, 0.0, true, 0.0, 0.0);
    System.out.println("swerve atDriveGoal: " + swerve.atDriveGoal());
    System.out.println("swerve atPathEndpoint: " + swerve.atPathEndpoint(0));
    System.out.println("swerve getAngVel: " + swerve.getAngVel());
    System.out.println("swerve getCalibrationTimer: " + swerve.getCalibrationTimer());
    System.out.println("swerve getAccurateCalibrationTimer: " + swerve.getAccurateCalibrationTimer());
    System.out.println("swerve getFusedAng: " + swerve.getFusedAng());
    System.out.println("swerve getGyroAng: " + swerve.getGyroAng());
    System.out.println("swerve getGyroPitch: " + swerve.getGyroPitch());
    System.out.println("swerve getGyroRoll: " + swerve.getGyroRoll());
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
    swerve.calcPriorityLimelightIndex();
    System.out.println("swerve getPriorityLimelightIndex: " + swerve.getPriorityLimelightIndex());
    swerve.updateDash();

    updateDash();
    calcScoringPoses();
    calcNearestScoringPose();
  }
}