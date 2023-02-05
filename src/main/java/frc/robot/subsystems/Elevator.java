// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.JoystickConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //button ports: A – 1, B – 2, X – 3, Y – 4

  //below double is to test elevator heights, can be removed
  private double lowerElevator = 100;
  private double midElevator = 500;
  private double actualPosition;
  private boolean elevatorState;
  private final XboxController m_driverController = new XboxController(JoystickConstants.DRIVER_PORT_ID);
  TalonSRX elevator = new TalonSRX(7);
  
  //rev hex bore thru half inch, 8000 ticks per revolution 1:1 scale w axle

//elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
//elevator.configFactoryDefault();
//elevator.setSensorPhase(true);

  public Elevator() {


    //elevator - chain is LOOSE
    elevatorState = m_driverController.getRawButton(2);
    actualPosition = elevator.getSelectedSensorPosition() * -1;

    if(actualPosition < (lowerElevator) && elevatorState){
      elevator.set(ControlMode.PercentOutput, 1);
      System.out.println(elevator.getSelectedSensorPosition());
    }

   if(actualPosition > lowerElevator && elevatorState) {
       elevator.set(ControlMode.PercentOutput, -1);
       System.out.println(elevator.getSelectedSensorPosition());
    }

    //elevator- manual 
    double elevator2 = m_driverController.getRawAxis(1);
    elevator.set(ControlMode.PercentOutput, elevator2);
    System.out.print(elevator.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
