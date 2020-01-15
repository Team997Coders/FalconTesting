/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private double gearingFactor = 0.12282854 * ((double)5.5 * Math.PI) / (double)12
   / (double)2048;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    l1 = new TalonFX(4);
    l2 = new TalonFX(5);
    r1 = new TalonFX(6);
    r2 = new TalonFX(7);

    l1.configFactoryDefault(10);
    l2.configFactoryDefault(10);
    r1.configFactoryDefault(10);
    r2.configFactoryDefault(10);

    l1.setInverted(false);
    l2.setInverted(false);
    r1.setInverted(true);
    r2.setInverted(true);

    l2.follow(l1);
    r2.follow(r1);

    l1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    r1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
    l1.setSelectedSensorPosition(0, 0, 10);
    r1.setSelectedSensorPosition(0, 0, 10);

    controller = new XboxController(0);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  private double maxSpeed = 0;

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Drive/Left1 Temp", l1.getTemperature());
    SmartDashboard.putNumber("Drive/Left2 Temp", l2.getTemperature());
    SmartDashboard.putNumber("Drive/Right1 Temp", r1.getTemperature());
    SmartDashboard.putNumber("Drive/Right2 Temp", r2.getTemperature());

    //SmartDashboard.putNumber("Drive/Left Pos", l1.getSelectedSensorPosition(0));
    //SmartDashboard.putNumber("Drive/Right Pos", r1.getSelectedSensorPosition(0));
    //SmartDashboard.putNumber("Drive/Left Vel", l1.getSelectedSensorVelocity(0));
    //SmartDashboard.putNumber("Drive/Right Vel", r1.getSelectedSensorVelocity(0));

    SmartDashboard.putNumber("Drive/Left Ft/s", getAlteredSpeed(l1));
    SmartDashboard.putNumber("Drive/Right Ft/s", getAlteredSpeed(r1));
    SmartDashboard.putNumber("Drive/Left Ft", getAlteredPosition(l1));
    SmartDashboard.putNumber("Drive/Right Ft", getAlteredPosition(r1));
    
    double pos = l1.getSelectedSensorPosition(0);//((double)l1.getSelectedSensorPosition(0)) * (1 / 2048) * gearingFactor * (6 / 12);
    pos /= 2048;
    pos *= gearingFactor;
    pos *= ((double)5.5 * Math.PI) / (double)12;
    System.out.println("Pos: " + pos);
    //SmartDashboard.putNumber("Drive/Right Ft", getAlteredPosition(false));

    if (l1.getSelectedSensorVelocity(0) > maxSpeed) maxSpeed = l1.getSelectedSensorVelocity(0);
    SmartDashboard.putNumber("Drive/MaxSpeed (Left)", maxSpeed);
  }

  public double getAlteredSpeed(TalonFX motor) {
    double a = (double)motor.getSelectedSensorVelocity(0);
    a *= (double)10;
    return a * gearingFactor;
  }

  public double getAlteredPosition(TalonFX motor) {
    double a = (double)motor.getSelectedSensorPosition(0);
    return a * gearingFactor;
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
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

    double f = -controller.getRawAxis(1);
    double t = controller.getRawAxis(4);

    double left = f + t;
    double right = f - t;

    l1.set(ControlMode.PercentOutput, left);
    r1.set(ControlMode.PercentOutput, right);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
