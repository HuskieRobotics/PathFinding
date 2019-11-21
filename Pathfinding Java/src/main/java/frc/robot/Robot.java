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




public class Robot extends TimedRobot {

  private static final int k_ticks_per_rev = 28;
  private static final double k_wheel_diameter = .5;
  private static final double k_max_velocity = 4;

  private CANSparkMax[] leftMotors;
  private CANSparkMax[] rightMotors;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  // private static final int k_gyro_port = 0;

  private static final String k_path_name = "/home/lvuser/deploy/paths/Unnamed";

  private SpeedController left_motors;

  private SpeedController right_motors;
  private static final int[] leftDeviceIDs = {1,2,3};
  private static final int[] rightDeviceIDs = {4,5,6};

  // private AnalogGyro m_gyro;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  
  private Notifier m_follower_notifier;



  @Override
  public void robotInit() {
    for(int i=0;i<3;i++)
    {
      leftMotors[i] = new CANSparkMax(leftDeviceIDs[i], MotorType.kBrushless);
      leftMotors[i].restoreFactoryDefaults();
      if(leftDeviceIDs[i]==2)
      {
        leftMotors[i].setInverted(true);
      }
      rightMotors[i] = new CANSparkMax(rightDeviceIDs[i], MotorType.kBrushless);
      rightMotors[i].restoreFactoryDefaults();
      if(rightDeviceIDs[i]==4 || rightDeviceIDs[i]==6)
      {
        rightMotors[i].setInverted(true);
      }
    }

    leftEncoder = leftMotors[0].getEncoder(EncoderType.kQuadrature,4096);
    rightEncoder = rightMotors[0].getEncoder(EncoderType.kQuadrature,4096);

    // m_gyro = new AnalogGyro(k_gyro_port);

  }





  @Override
  public void robotPeriodic() {
  }




  @Override
  public void autonomousInit() {

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
      // double heading = m_gyro.getAngle();
      // double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      // double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      // double turn =  0.8 * (-1.0/80.0) * heading_difference;
      // for(CANSparkMax leftMotor:leftMotors) {leftMotor.set(left_speed + turn);}
      // for(CANSparkMax rightMotor:rightMotors) {rightMotor.set(right_speed + turn);}
    }
  }

  

  @Override
  public void autonomousPeriodic() {
    this.followPath();
  }


  @Override
  public void teleopInit() {
    m_follower_notifier.stop();
    for(CANSparkMax leftMotor:leftMotors) {leftMotor.set(0);}
    for(CANSparkMax rightMotor:rightMotors) {rightMotor.set(0);}
  }




   @Override
  public void teleopPeriodic() {
  }



  @Override
  public void testPeriodic() {
  }
}
