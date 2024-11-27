// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Robot extends TimedRobot {

  private CANSparkMax leftMotor1 = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(3, MotorType.kBrushless);

  private XboxController xbox1 = new XboxController(0);

  Encoder ChasisEncodeR = new Encoder(6, 7);
  Encoder ChasisEncodeL = new Encoder(8, 9); 

  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  @Override
  public void robotInit() {}

  

  @Override
  public void autonomousInit() {
    ChasisEncodeR.reset();
    errorSum = 0;
    lastError = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  } 

  final double kP = 0.05;
  final double kI = 0.5;
  final double kD = 0.01;
  final double iLimit = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimestamp = 0;
  double lastError = 0;

  @Override
  public void autonomousPeriodic() {
    //get joystick command
    if(xbox1.getRawButton(1)) {
        setpoint = 10;
    }else if (xbox1.getRawButton(2)) {
        setpoint = 0;
    }

    // get sensor position
    double sensorPosition = ChasisEncodeR.get() * kDriveTick2Feet;

    //calculations
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if(Math.abs(error) < iLimit) {
        errorSum += error * dt;
    }
    
    double errorRate = (error - lastError) / dt;
    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    //output to motors
    leftMotor1.set(outputSpeed);
    leftMotor2.set(outputSpeed);
    rightMotor1.set(-outputSpeed);
    rightMotor2.set(-outputSpeed);

    //update last- variables
    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", ChasisEncodeR.get() * kDriveTick2Feet);
  }
  
  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
