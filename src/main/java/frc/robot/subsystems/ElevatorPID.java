// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorPID extends PIDSubsystem {
  /** Creates a new ElevatorPID. */

  //write commands to go to different heights; three points, 200, 4000, 6000
  private static final int deviceID = 1;
  private CANSparkMax m_elevator;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  public ElevatorPID() {

    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        
    m_elevator = new CANSparkMax(0, MotorType.kBrushless);
    
     // PID coefficients
     kP = 5e-5; 
     kI = 1e-6;
     kD = 0; 
     kIz = 0; 
     kFF = 0.000156; 
     kMaxOutput = 1; 
     kMinOutput = -1;
     maxRPM = 5700;
 
     // Smart Motion Coefficients
     maxVel = 2000; // rpm
     maxAcc = 1500;
 
     // set PID coefficients
     m_pidController.setP(kP);
     m_pidController.setI(kI);
     m_pidController.setD(kD);
     m_pidController.setIZone(kIz);
     m_pidController.setFF(kFF);
     m_pidController.setOutputRange(kMinOutput, kMaxOutput);

     int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }
  }


  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
