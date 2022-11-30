

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.motorcontrol.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


//change the motor controller to can spark max and make alsve and master motors drive in diffrentiel drive
//and find the field in simulations
public class Robot extends TimedRobot {
  private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(0);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(3);
  private TalonSRX armMotor = new TalonSRX(4); 
  private TalonSRX armSlave = new TalonSRX(5); 
  private TalonSRX rollerMotor = new TalonSRX(6);
  private Compressor compressor = new Compressor(null);
  private DoubleSolenoid hatchIntake  = new DoubleSolenoid(null, 0, 1);
  private Joystick driverJoystick = new Joystick(7);
  private Joystick operaJoystick = new Joystick(8);
  //we will get to this at the end
  private DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);
  //tickes to feet ( after encoders i will go over it)
  private final double kDriveTick2Feet= 1.0/4096 * 6 * Math.PI / 12;
  //tickes to degrees 
  private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
  
  @Override
  public void robotInit() { 
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);
    armSlave.follow(armMotor);
    //init encoders                           //type of encoder       //pid(not for now)    //timeout(if faild after 10 ms retry sending feedback)
    leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    //sets right to positive like the inverted
    rightMotor1.setSensorPhase(true);
    armMotor.setSensorPhase(true);
    //reset encoder values to 0     //sensorpos  //pid   //timeout
    leftMotor1.setSelectedSensorPosition(0, 0, 10);
    rightMotor1.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);

    //i dont want the robot arm to overreach a deg so that not to break the robot and for that
    //i will be using the encoder
    //as to not get a negetive angle at the start well set the start at 0 deg and cant over come it and
    //if tried the robot will wait 10 ms before continueing
    armMotor.configReverseSoftLimitThreshold((int) 0 / kArmTick2Deg, 10);
    //deg to tickes
    armMotor.configForwardSoftLimitThreshold((int) 175 / kArmTick2Deg, 10);
    //lets enable
    armMotor.configReverseSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);

    //start compressor when robot starts
    compressor.enabled();
    //we can also set deadband like so
    drive.setDeadband(0.05);
  }
 
  @Override
  public void robotPeriodic() {
    //displays to the smart dashboard the sensor position in deg (deg motor turned)
    SmartDashboard.putNumber("arm encoder value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    //displays to the smart dashboard the sensor position in feet (distance motor turned)
    SmartDashboard.putNumber("left drive Encoder value", leftMotor1.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("right drive Encoder value", rightMotor1.getSelectedSensorPosition() * kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {
    //reset incoders
    leftMotor1.setSelectedSensorPosition(0, 0, 10);
    rightMotor1.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);
     
    enableMotors(true);

  }
  
  @Override
  public void autonomousPeriodic() {
    //get init pos of the motors
   double leftpos = leftMotor1.getSelectedSensorPosition() * kDriveTick2Feet;
   double rightpos = rightMotor1.getSelectedSensorPosition() * kDriveTick2Feet;
   //dis travled is avg of both motors
   double distance = (leftpos + rightpos) / 2;
   if(distance < 10){
     //drive robot forward at 60% speed
     drive.tankDrive(0.6, 0.6);
   }
   else {
     drive.tankDrive(0, 0);
   }
  }
   
  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
  //                             between 1 and -1 
  double power = -driverJoystick.getRawAxis(1);
  double turn = driverJoystick.getRawAxis(4);

  //seting deadband (the joystick does not come back all the way and the motor can still spin there for we will
  //set that if the joystick does not come all the way as indicated by the power output the power will be set to 0)
   if(Math.abs(power) < 0.05){
     power = 0;
    }
    if(Math.abs(turn) < 0.05){
     turn = 0;
    }
    //but we can delete this because this feature is built in to the diff drive lib
    drive.arcadeDrive(power, turn);
    
   //arm controll
   double armPower = -operaJoystick.getRawAxis(1);
   if(Math.abs(armPower) < 0.05){
    armPower = 0;
   }
   armPower = armPower * 0.5;
    //the percent output from the talon 
   //the talon mode indicates in what for we will get the value
   //still not sure why its needed i will ask and update u after the meating
    armMotor.set(ControlMode.PercentOutput, armPower);
     
    //roller control
    double rollerPower = 0;
    if(operaJoystick.getRawButton(1) == true){
      rollerPower =1;
    }
    rollerMotor.set(ControlMode.PercentOutput, rollerPower);

    //hatch intake
    if(operaJoystick.getRawButton(3) == true){
      //open solenoid
      hatchIntake.set(Value.kReverse);
    }
    else
    {
      //close solenoid
      hatchIntake.set(Value.kForward);
    }
  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  
  @Override
  public void simulationInit() { System.out.println("is simulating");}

  @Override
  public void simulationPeriodic() {}

  private void enableMotors(boolean on)
  {
    //break mode helps arm stay in place against gravity. u want it on when tobot is enabled
    //coast mode lets u rotate the object. u want it on only when the robot is disabled
    NeutralMode mode;
    if(on == true){
      mode = NeutralMode.Brake;
    }
    else {
      mode = NeutralMode.Coast;
    }
      leftMotor1.setNeutralMode(mode);
      leftMotor2.setNeutralMode(mode);
      rightMotor1.setNeutralMode(mode);
      rightMotor2.setNeutralMode(mode);
      armMotor.setNeutralMode(mode);
      armSlave.setNeutralMode(mode);
      rollerMotor.setNeutralMode(mode);
  }
}
