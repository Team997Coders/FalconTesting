/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private TalonFX l1, l2, r1, r2;
  private XboxController controller;
  private double targetSpeed = 0.0;
  
  private boolean followingPath = false;
  private double startTime = Double.NaN;
  private double lastTime;
  private Rotation2d lastRotation;
  private Translation2d lastTranslation;

  private double vel2FeetConversion;

  private double leftOutput = 0.0, rightOutput = 0.0;

  private Trajectory traTest;
  private DifferentialDriveKinematics diffDriveKin;

  private double gearingFactor = 0.12282854 * ((double) 5.5 * Math.PI) / (double) 12 / (double) 2048;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    try {
      Path p = Paths.get("/home/lvuser/deploy/paths/output/AHH.wpilib.json");
      traTest = TrajectoryUtil.fromPathweaverJson(p);
      double a = traTest.sample(0).accelerationMetersPerSecondSq;
      System.out.println("Acceleration M/s2 -> " + a);

      diffDriveKin = new DifferentialDriveKinematics(f2m(2.1));

    } catch (Exception e) { e.printStackTrace(); }

    vel2FeetConversion = 10;
    System.out.println(vel2FeetConversion);
    vel2FeetConversion *= ((double)1 / (double)2048);
    System.out.println(vel2FeetConversion);
    vel2FeetConversion *= ((double)9 / (double)70);
    System.out.println(vel2FeetConversion);
    vel2FeetConversion *= ((double)5 * Math.PI);
    System.out.println(vel2FeetConversion);
    vel2FeetConversion *= ((double)1 / (double)12);
    System.out.println(vel2FeetConversion);

    l1 = new TalonFX(4);
    l2 = new TalonFX(3);
    r1 = new TalonFX(1);
    r2 = new TalonFX(2);

    l1.configFactoryDefault(10);
    l2.configFactoryDefault(10);
    r1.configFactoryDefault(10);
    r2.configFactoryDefault(10);

    l1.setInverted(true);
    l2.setInverted(true);
    r1.setInverted(false);
    r2.setInverted(false);

    l2.follow(l1);
    r2.follow(r1);

    l1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    r1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    l1.setSelectedSensorPosition(0, 0, 10);
    r1.setSelectedSensorPosition(0, 0, 10);

    l1.configPeakOutputForward(1, 10);
    l1.configPeakOutputReverse(-1, 10);

    r1.configPeakOutputForward(1, 10);
    r1.configPeakOutputReverse(-1, 10);

    double p = 0.13;
    double d = (double)3;
    double f = (double)1023 / (double)19990;
    
    l1.config_kP(0, p, 10);
    //l1.config_kI(0, 0.000007, 10);
    l1.config_kD(0, d, 10);
    l1.config_kF(0, f, 10); // 1 / 21777 also 19990
    
    r1.config_kP(0, p, 10);
    r1.config_kD(0, d, 10);
    r1.config_kF(0, f, 10);
    
    controller = new XboxController(0);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    System.out.println("Followers initiated");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  private double maxSpeed = 0;

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Drive/Left1 Temp", l1.getTemperature());
    SmartDashboard.putNumber("Drive/Left2 Temp", l2.getTemperature());
    SmartDashboard.putNumber("Drive/Right1 Temp", r1.getTemperature());
    SmartDashboard.putNumber("Drive/Right2 Temp", r2.getTemperature());

    // SmartDashboard.putNumber("Drive/Left Pos", l1.getSelectedSensorPosition(0));
    // SmartDashboard.putNumber("Drive/Right Pos", r1.getSelectedSensorPosition(0));
    // SmartDashboard.putNumber("Drive/Left Vel", l1.getSelectedSensorVelocity(0));
    // SmartDashboard.putNumber("Drive/Right Vel", r1.getSelectedSensorVelocity(0));

    SmartDashboard.putNumber("Drive/Left Ft/s", getAlteredSpeed(l1));
    //SmartDashboard.putNumber("Drive/Left Ft/s", l1.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Drive/Right Ft/s", getAlteredSpeed(r1));
    SmartDashboard.putNumber("Drive/Left Ft", getAlteredPosition(l1));
    SmartDashboard.putNumber("Drive/Right Ft", getAlteredPosition(r1));

    SmartDashboard.putNumber("Auto/Left Output", leftOutput);
    SmartDashboard.putNumber("Auto/Right Output", rightOutput);

    double pos = l1.getSelectedSensorPosition(0);// ((double)l1.getSelectedSensorPosition(0)) * (1 / 2048) *
                                                 // gearingFactor * (6 / 12);
    pos /= 2048;
    pos *= gearingFactor;
    pos *= ((double) 5.5 * Math.PI) / (double) 12;
    //System.out.println("Pos: " + pos);
    // SmartDashboard.putNumber("Drive/Right Ft", getAlteredPosition(false));

    if (l1.getSelectedSensorVelocity(0) > maxSpeed)
      maxSpeed = l1.getSelectedSensorVelocity(0);
    SmartDashboard.putNumber("Drive/MaxSpeed (Left)", maxSpeed);
  }

  public double getAlteredSpeed(TalonFX motor) {
    //double a = (double) motor.getSelectedSensorVelocity(0);
    //a *= (double) 10;
    return vel2f(motor.getSelectedSensorVelocity(0));
  }

  public double getAlteredPosition(TalonFX motor) {
    return vel2f(motor.getSelectedSensorPosition(0)) / (double)10;
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (!followingPath) {
      if (controller.getAButtonPressed()) {
        l1.setNeutralMode(NeutralMode.Coast);
        l2.setNeutralMode(NeutralMode.Coast);
        r1.setNeutralMode(NeutralMode.Coast);
        r2.setNeutralMode(NeutralMode.Coast);
      }
      if (controller.getBButtonPressed()) {
        l1.setNeutralMode(NeutralMode.Brake);
        l2.setNeutralMode(NeutralMode.Brake);
        r1.setNeutralMode(NeutralMode.Brake);
        r2.setNeutralMode(NeutralMode.Brake);
      }

      if (controller.getStartButtonPressed()) {
        followingPath = true;
      }

      double f = -controller.getRawAxis(1);
      double t = controller.getRawAxis(4);

      double left = f + t;
      double right = f - t;

      l1.set(ControlMode.PercentOutput, left);
      r1.set(ControlMode.PercentOutput, right);
    } else {
      if (!Double.isFinite(startTime)) {
        startTime = (double)System.currentTimeMillis() / (double)1000;

        lastTime = startTime;
        lastRotation = traTest.getInitialPose().getRotation();
        lastTranslation = traTest.getInitialPose().getTranslation();

        l1.setSelectedSensorPosition(0, 0, 10);
        r1.setSelectedSensorPosition(0, 0, 10);
      }

      double currentTime = (double)System.currentTimeMillis() / (double)1000;

      Trajectory.State currentState = traTest.sample(currentTime - startTime);
      Translation2d translation = currentState.poseMeters.getTranslation();
      Rotation2d rotation = currentState.poseMeters.getRotation();
      SmartDashboard.putNumber("Auto/Translation X", translation.getX());
      SmartDashboard.putNumber("Auto/Translation Y", translation.getY());
      SmartDashboard.putNumber("Auto/Translation", translation.getNorm());
      SmartDashboard.putNumber("Auto/Velocity", currentState.velocityMetersPerSecond);

      Translation2d diffTrans = translation.minus(lastTranslation);
      Rotation2d diffRota = rotation.minus(lastRotation);
      double deltaT = currentTime - lastTime;
      //diffTrans = diffTrans.div(deltaT);

      //double xV = diffTrans.getX() / deltaT;
      //double yV = diffTrans.getY() / deltaT;
      double x = (diffTrans.getX() / diffTrans.getNorm()) * currentState.velocityMetersPerSecond;
      double y = (diffTrans.getY() / diffTrans.getNorm()) * currentState.velocityMetersPerSecond;
      double rV = currentState.curvatureRadPerMeter * currentState.velocityMetersPerSecond;
      //currentState.

      ChassisSpeeds uhoh = new ChassisSpeeds(currentState.velocityMetersPerSecond, 0, rV);
      //ChassisSpeeds uhoh = new ChassisSpeeds(diffTrans.getX(), diffTrans.getY(), rV);
      DifferentialDriveWheelSpeeds wheelSpeeds = diffDriveKin.toWheelSpeeds(uhoh);
      double leftFtPS = m2f(wheelSpeeds.leftMetersPerSecond);
      double rightFtPS = m2f(wheelSpeeds.rightMetersPerSecond);

      //leftOutput = f2vel(5);
      //rightOutput = f2vel(5);
      leftOutput = f2vel(leftFtPS);
      rightOutput = f2vel(rightFtPS);

      //double speedToOutputConversion = 1 / ()

      SmartDashboard.putNumber("Auto/Left Ft s", leftFtPS);
      SmartDashboard.putNumber("Auto/Right Ft s", rightFtPS);

      //System.out.println("LeftFtPS: " + leftFtPS);
      //System.out.println("RightFtPS: " + rightFtPS);

      l1.set(ControlMode.Velocity, leftOutput);
      r1.set(ControlMode.Velocity, rightOutput);

      SmartDashboard.putNumber("Auto/LeftPIDOut", l1.getClosedLoopError(0));
      SmartDashboard.putNumber("Auto/RightPIDOut", r1.getClosedLoopError(0));

      if (currentState == traTest.getStates().get(traTest.getStates().size() - 1)) {
        startTime = Double.NaN;
        followingPath = false;
      }

      lastRotation = currentState.poseMeters.getRotation();
      lastTranslation = currentState.poseMeters.getTranslation();
    }
  }

  public double vel2f(double vel) {
    return (double)vel * (double)vel2FeetConversion;
  }

  public double f2vel(double feet) {
    System.out.println("F2Vel conversion = " + (1 / vel2FeetConversion));
    return (double)feet / (double)vel2FeetConversion;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private double f2m(double feet) {
    return feet / 3.281;
  }

  private double m2f(double meters) {
    return meters * 3.281;
  }
}
