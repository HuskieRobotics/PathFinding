/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.EncoderType;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

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

  private static final int k_ticks_per_rev = 4096;
  private static final double k_wheel_diameter = .5;
  private static final double k_max_velocity = 4;

  private CANSparkMax[] leftMotors;
  private CANSparkMax[] rightMotors;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  private static final int k_gyro_port = 0;

  private static final String k_path_name = "/home/lvuser/deploy/paths/Unnamed";

  private SpeedController left_motors;
  private SpeedController right_motors;
  private static final int[] leftDeviceIDs = {1,2,3};
  private static final int[] rightDeviceIDs = {4,5,6};

  private AnalogGyro m_gyro;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  
  private Notifier m_follower_notifier;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    for(int i=0;i<3;i++)
    {
      leftMotors[i] = new CANSparkMax(leftDeviceIDs[i], MotorType.kBrushless);
      if(leftDeviceIDs[i]==2)
      {
        leftMotors[i].setInverted(true);
      }
      rightMotors[i] = new CANSparkMax(rightDeviceIDs[i], MotorType.kBrushless);
      if(rightDeviceIDs[i]==4 || rightDeviceIDs[i]==6)
      {
        rightMotors[i].setInverted(true);
      }
      leftMotors[i].restoreFactoryDefaults();
      rightMotors[i].restoreFactoryDefaults();
    }

    leftEncoder = leftMotors[0].getEncoder(EncoderType.kQuadrature,4096);
    rightEncoder = rightMotors[0].getEncoder(EncoderType.kQuadrature,4096);

    m_gyro = new AnalogGyro(k_gyro_port);

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
  @Override
  public void robotPeriodic() {
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

    Trajectory left_trajectory = null;
    Trajectory right_trajectory = null;
    try{
      left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
      right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");}
    catch(IOException e){
      System.out.println("no file found");
    }

    m_left_follower = new EncoderFollower(left_trajectory);
    m_right_follower = new EncoderFollower(right_trajectory);

    m_left_follower.configureEncoder((int) leftEncoder.getPosition(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    m_right_follower.configureEncoder((int) rightEncoder.getPosition(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    
    m_follower_notifier = new Notifier(this::followPath);
    m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
  }
  
  private void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate((int) leftEncoder.getPosition());
      double right_speed = m_right_follower.calculate((int) rightEncoder.getPosition());
      double heading = m_gyro.getAngle();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      for(CANSparkMax leftMotor:leftMotors) {leftMotor.set(left_speed + turn);}
      for(CANSparkMax rightMotor:rightMotors) {rightMotor.set(right_speed + turn);}
    }
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


  @Override
  public void teleopInit() {
    m_follower_notifier.stop();
    for(CANSparkMax leftMotor:leftMotors) {leftMotor.set(0);}
    for(CANSparkMax rightMotor:rightMotors) {rightMotor.set(0);}
  }


  /**
   * This function is called periodically during operator control.
   */  
   @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
