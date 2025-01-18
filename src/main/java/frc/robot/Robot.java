package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
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

  private double speedScaleFactor = 1.0; // Scales the speed of the robot that results from controller inputs. 1.0 corresponds to full speed. 0.0 is fully stopped.
  private boolean lock = false; // Controls whether the swerve drive is in x-lock (for defense) or is driving. \

  PIDController pid = new PIDController(0.1, 0.0, 0.0); // Initializes a PID controller with a P value of 0.1, an I value of 0.0, and a D value of 0.0.

  // Initializes the different subsystems of the robot.
  private final Drivetrain swerve = new Drivetrain(); // Contains the Swerve Modules, Gyro, Path Follower, Target Tracking, Odometry, and Vision Calibration.
  private final Elevator elevator = new Elevator(); // Contains the elevator motor and limit switch.
  //DigitalInput limitSwitch = new DigitalInput(0); // Initializes the sensor connected to DIO port 0 on the RoboRIO.
  
  // Auto Variables
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private static final String auto1 = "auto1";
  private static final String auto2 = "auto2";
  private String autoSelected;
  private int autoStage = 1;

  private final ProfiledPIDController alignXController = new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(Drivetrain.maxAngVelAuto, Drivetrain.maxAngAccAuto));
  private final ProfiledPIDController alignYController = new ProfiledPIDController(0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(Drivetrain.maxAngVelAuto, Drivetrain.maxAngAccAuto));
  private final ProfiledPIDController alignAngleController = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Drivetrain.maxAngVelAuto, Drivetrain.maxAngAccAuto));
  private final double posTol = 0.03;
  private final double angTol = 0.5;
  boolean atAlignTarget = false;

  // Should be called immediately prior to alignToTag(). Resets the PID controllers. Target angle specifies the angle that will be demanded in alignToTag().
  public void resetAlignController(double angleTarget) {
    double TX = LimelightHelpers.getTX(swerve.limelights[0]);
    double TY = LimelightHelpers.getTY(swerve.limelights[0]);
    alignAngleController.reset(swerve.getAngleDistance(swerve.getFusedAng(), angleTarget)*Math.PI/180.0, 0.0);
    alignXController.reset(TX, 0.0);
    alignYController.reset(TY, 0.0);
    atAlignTarget = false;
  }

  // Aligns the robot to an April Tag. 
  // xTarget represents the AprilTag's target x-coordinate on the Limelight feed. 
  // yTarget represents the AprilTag's target y-coordinate on the Limelight feed.
  // angleTarget represents the robot's target heading.
  public void alignToTag(double xTarget, double yTarget, double angleTarget) {
    double TX = LimelightHelpers.getTX(swerve.limelights[0]);
    double TY = LimelightHelpers.getTY(swerve.limelights[0]);
    double angleDistance = swerve.getAngleDistance(swerve.getFusedAng(), angleTarget);
    double angVelSetpoint = alignAngleController.calculate(angleDistance*Math.PI/180.0, 0.0);
    double xVelSetpoint = alignXController.calculate(TX, xTarget);
    double yVelSetpoint = alignYController.calculate(TY, yTarget);
    boolean atAngTarget = Math.abs(angleDistance) < angTol;
    boolean atXTarget = Math.abs(TX - xTarget) < posTol;
    boolean atYTarget = Math.abs(TY - yTarget) < posTol;

    // Checks to see if all 3 targets have been achieved. Sets velocities to 0 to prevent twitchy robot motions at near 0 velocities.
    atAlignTarget = atXTarget && atYTarget && atAngTarget;
    if (atAngTarget) angVelSetpoint = 0.0;
    if (atXTarget) xVelSetpoint = 0.0;
    if (atYTarget) yVelSetpoint = 0.0;

    // Caps the velocities if the PID controllers return values above the specified maximums.
    if (Math.abs(xVelSetpoint) > Drivetrain.maxVelAuto) {
      xVelSetpoint = xVelSetpoint > 0.0 ?  Drivetrain.maxVelAuto : -Drivetrain.maxVelAuto;
    }
    if (Math.abs(yVelSetpoint) > Drivetrain.maxVelAuto) {
      yVelSetpoint = yVelSetpoint > 0.0 ? Drivetrain.maxVelAuto : -Drivetrain.maxVelAuto;
    }
    if (Math.abs(angVelSetpoint) > Drivetrain.maxAngVelAuto) {
      angVelSetpoint = angVelSetpoint > 0.0 ? Drivetrain.maxAngVelAuto : -Drivetrain.maxAngVelAuto;
    }

    swerve.drive(xVelSetpoint, yVelSetpoint, angVelSetpoint, false, 0.0, 0.0);
  }

  // Returns true if the robot is aligned to the April Tag.
  public boolean atAlignTarget() {
    return atAlignTarget;
  }

/*
  public void twoStageControl(double robotX,double robotY,double robotAngle) {
    double TX = LimelightHelpers.getTX(swerve.limelights[0]);
    double TY = LimelightHelpers.getTY(swerve.limelights[0]);
    double xSetpoint = pid.calculate();
    double ySetpoint = pid.calculate();
    double angleSetpoint = pid.calculate();


    //Stage 1
    swerve.driveTo(robotX, robotY, robotAngle);

    if (swerve.atDriveGoal()) {
      aimEnd = true;
    }
    //Stage 2
    if (Math.abs(xSetpoint) > Drivetrain.maxAngVelAuto) {
      xSetpoint = xSetpoint > 0.0 ?  Drivetrain.maxAngVelAuto : -Drivetrain.maxAngVelAuto;
    }
    if (Math.abs(ySetpoint) > Drivetrain.maxAngVelAuto) {
      ySetpoint = ySetpoint > 0.0 ? Drivetrain.maxAngVelAuto : -Drivetrain.maxAngVelAuto;
    }

    if (aimEnd) {
      if (TX > 1.8) {
        swerve.drive(0.0, xSetpoint, 0.0, true, 0.0, 0.0);
      }
    }

    if (aimEnd){
      if (TY > 1.8) {
        swerve.drive(ySetpoint, 0.0, 0.0, true, 0.0, 0.0);
      } 
    }
    
  }
    */

  public void robotInit() { 
    // Configures the auto chooser on the dashboard.
    autoChooser.setDefaultOption(auto1, auto1);
    autoChooser.addOption(auto2, auto2);
    SmartDashboard.putData("Autos", autoChooser);

    swerve.loadPath("Test", 0.0, 0.0, 0.0, 0.0); // Loads a Path Planner generated path into the path follower code in the drivetrain. 
    runAll(); // Helps prevent loop overruns on startup by running every command before the match starts.
  }

  public void robotPeriodic() {
    // Publishes information about the robot and robot subsystems to the Dashboard.
    swerve.updateDash();
    elevator.updateDash();
    updateDash();
    swerve.updateVisionHeading(); // Updates the Limelights with the robot heading (for MegaTag2).
    if (driver.getRawButtonPressed(8)) swerve.resetGyro(); // Menu Button re-zeros the angle reading of the gyro to the current angle of the robot. Should be called if the gyroscope readings are no longer well correlated with the field.
  }

  public void autonomousInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
    autoStage = 1;
    autoSelected = autoChooser.getSelected();
    switch (autoSelected) {
      case auto1:
        // AutoInit 1 code goes here.
      break;

      case auto2:
        // AutoInit 2 code goes here.
      break;
    }
  }

  public void autonomousPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    switch (autoSelected) {
      case auto1:
        switch (autoStage) {
          case 1:
            // Auto 1, Stage 1 code goes here.
            swerve.drive(0.0, 0.0, 0.0, true, 0.0, 0.0);
            elevator.setElevatorPosition(3.0);
            if (elevator.isAtSetpoint()) {
              autoStage = -1; // Goes to default case.
            }
          break;

          case 2:
            // Auto 1, Stage 2 code goes here.
          break;

        default:
          elevator.setElevatorPosition(0.0);
          break;

        }
      break;

      case auto2:
        switch (autoStage) {
          case 1:
            // Auto 2, Stage 1 code goes here.
          break;

          case 2:
            // Auto 2, Stage 2 code goes here.
          break;
        }
      break;
    }
  }
  
  public void teleopInit() {
    swerve.pushCalibration(); // Updates the robot's position on the field.
  }

  public void teleopPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addVisionEstimate(limelightIndex, 0.7, 0.7, Units.degreesToRadians(Math.pow(10, 10)), true); // Checks to see ifs there are reliable April Tags in sight of the Limelight and updates the robot position on the field.
    }

    if (driver.getRawButtonPressed(4)) speedScaleFactor = 1.0; // Y Button sets the drivetrain in full speed mode.
    if (driver.getRawButtonPressed(2)) speedScaleFactor = 0.6; // B button sets the drivetrain in medium speed mode.
    if (driver.getRawButtonPressed(1)) speedScaleFactor = 0.15; // A button sets the drivetrain in low speed mode.

    elevator.manualElevator(MathUtil.applyDeadband(-operator.getLeftY(), 0.1)); // Controls the elevator with the left joystick on the operator controller.

    // Applies a deadband to controller inputs. Also limits the acceleration of controller inputs.
    double xVel = xAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftY(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double yVel = yAccLimiter.calculate(MathUtil.applyDeadband(-driver.getLeftX(), 0.05)*speedScaleFactor)*Drivetrain.maxVelTeleop;
    double angVel = angAccLimiter.calculate(MathUtil.applyDeadband(-driver.getRightX(), 0.05)*speedScaleFactor)*Drivetrain.maxAngVelTeleop;

    if (driver.getRawButton(3)) {
      lock = true; // Pressing the X-button causes the swerve modules to lock (for defense).
    } else if (Math.abs(driver.getLeftY()) >= 0.05 || Math.abs(driver.getLeftX()) >= 0.05 || Math.abs(driver.getRightX()) >= 0.05) {
      lock = false; // Pressing any joystick more than 5% will cause the swerve modules stop locking and begin driving.
    }

    if (lock) {
      swerve.xLock(); // Locks the swerve modules (for defense).
    } else {
      swerve.drive(xVel, yVel, angVel, false, 0.0, 0.0); // Drive at the velocity demanded by the controller.
    }

    // The following 3 calls allow the user to calibrate the position of the robot based on April Tag information. Should be called when the robot is stationary. Button 7 is "View", the right center button.
    if (driver.getRawButtonPressed(7)) swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
    if (driver.getRawButton(7)) {
      for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
        swerve.addCalibrationEstimate(limelightIndex, false); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
      }
    }
    if (driver.getRawButtonReleased(7)) swerve.pushCalibration(); // Updates the position of the robot on the field based on previous calculations.  
  }

  public void disabledInit() {    
    swerve.resetCalibration(); // Begins calculating the position of the robot on the field based on visible April Tags.
  }

  public void disabledPeriodic() {
    swerve.updateOdometry(); // Keeps track of the position of the robot on the field. Must be called each period.
    for (int limelightIndex = 0; limelightIndex < swerve.limelights.length; limelightIndex++) { // Iterates through each limelight.
      swerve.addCalibrationEstimate(limelightIndex, false); // Collects additional data to calculate the position of the robot on the field based on visible April Tags.
    }
  }

  // Publishes information to the dashboard.
  public void updateDash() {
    //SmartDashboard.putNumber("Speed Scale Factor", speedScaleFactor);
    //SmartDashboard.putNumber("Auto Stage", autoStage);
  }

  // Helps prevent loop overruns on startup by running every user created command in every class before the match starts. Not sure why this helps, but it does.
  public void runAll() { 
    swerve.resetDriveController(0.0);
    swerve.xLock();
    swerve.aimDrive(-3.0, 2.0, 105.0, false);
    swerve.driveTo(1.0, -2.0, -75.0);
    swerve.resetPathController(0);
    swerve.followPath(0);
    swerve.pushCalibration();
    swerve.addCalibrationEstimate(0, false);
    swerve.pushCalibration();
    swerve.resetCalibration();
    swerve.resetGyro();
    swerve.updateVisionHeading();
    swerve.addVisionEstimate(0, 0.7, 0.7, Units.degreesToRadians(Math.pow(10, 10)), true);
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
    swerve.updateDash();
    updateDash();
  }
}