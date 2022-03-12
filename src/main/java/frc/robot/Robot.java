

//importing librarys
package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

public class Robot extends TimedRobot {
  
  //variables
  //leftStick is literally the whole controller
  private DifferentialDrive m_myRobot;
  private Joystick leftStick;
  private static final int leftDeviceID1 = 3;
  private static final int leftDeviceID2 = 4;  
  private static final int rightDeviceID1 = 5;
  private static final int rightDeviceID2 = 6;
  private static final int armDeviceID1 = 7;
  private static final int intakeDeviceId1 = 8;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor2;
  private CANSparkMax m_armMotor1;
  private CANSparkMax m_intakeMotor1;
  private int previousPOV;
  private double previousThrottle, idleThrottle;
  private DifferentialDrivetrainSim m_driveSim;
  private MotorControllerGroup m_left,m_right;
  @Override

  //assigning variables
  public void robotInit() {
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushless);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushless);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushless);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushless);
    m_armMotor1 = new CANSparkMax(armDeviceID1,MotorType.kBrushless);
    m_intakeMotor1 = new CANSparkMax(intakeDeviceId1,MotorType.kBrushless);
    m_intakeMotor1.setSmartCurrentLimit(30);
    m_left = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    m_right = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    
    m_leftMotor1.restoreFactoryDefaults();
    m_rightMotor1.restoreFactoryDefaults();
    m_leftMotor2.restoreFactoryDefaults();
    m_rightMotor2.restoreFactoryDefaults();
    m_armMotor1.restoreFactoryDefaults();

    m_right.setInverted(true);

    m_armMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_armMotor1.setSoftLimit(SoftLimitDirection.kForward, 0);
    m_armMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armMotor1.setSoftLimit(SoftLimitDirection.kReverse, (float)-12.67);
    m_intakeMotor1.restoreFactoryDefaults();

    m_myRobot = new DifferentialDrive(m_left, m_right);

    leftStick = new Joystick(0);
    idleThrottle = leftStick.getRawAxis(1);

  }

  //this is for testing, it doesn't do anything
  public void simulationPeriodic() {
  // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  m_driveSim.setInputs(m_left.get() * RobotController.getInputVoltage(),m_right.get() * RobotController.getInputVoltage());
  // Advance the model by 20 ms. Note that if you are running this
  //  subsystem in a separate thread or have changed the nominal timestep
  //  of TimedRobot, this value needs to match it.
  m_driveSim.update(0.02);
  }  

  @Override
  public void teleopPeriodic() {

    //driving
    m_myRobot.tankDrive(-1 * leftStick.getRawAxis(1), -1 * leftStick.getRawAxis(5));
    //m_myRobot.arcadeDrive(-1 * leftStick.getRawAxis(1), -1 * leftStick.getRawAxis(4));

    //this is for testing, it doesn't do anything
    if(leftStick.getPOV() != previousPOV)
    {
      System.out.println(Math.abs(leftStick.getPOV()-previousPOV)-1);
    }
    
    //intake
    // X is in, A is out
    if(leftStick.getRawButton(3))
      m_intakeMotor1.set(1);
    else if(leftStick.getRawButton(1))
      m_intakeMotor1.set(-1);
    else
    {
      System.out.println(m_intakeMotor1.get());
      m_intakeMotor1.set(0);
    }

    //this is for testing, it doesn't do anything
    if(leftStick.getRawAxis(1) != previousThrottle && idleThrottle != leftStick.getRawAxis(1))
    {
      System.out.println(leftStick.getRawAxis(1));
    }

    previousThrottle = leftStick.getRawAxis(1);
    previousPOV = leftStick.getPOV();

    //arm
    // 
    if(leftStick.getRawButton(2)) {
      m_armMotor1.set(1);
    }
    else if(leftStick.getRawButton(4)) {
      m_armMotor1.set(-1);   
    }
    else {
      m_armMotor1.set(0);
    }
      
    
    
    }

    public void autonomousPeriod() {
      
      m_intakeMotor1.set(-1);

      if(Timer.getMatchTime() < 14)  {
        m_intakeMotor1.set(0);
        while(Timer.getMatchTime() > 9) {
          m_myRobot.tankDrive(-1, -1);
        } 
      }
      
    }
  
      
}