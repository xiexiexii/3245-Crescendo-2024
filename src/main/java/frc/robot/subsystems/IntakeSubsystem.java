// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;
import frc.robot.Constants.SensorConstants;

public class IntakeSubsystem extends SubsystemBase {
    //init stuff
      // CANSparkMax intakeBottomMotor;
      CANSparkMax intakeTop1Motor;
      CANSparkMax intakeTop2Motor;
      DigitalInput beamBreak;
      TalonFX flipoutRun;
    
  public IntakeSubsystem() {
    //now falcons
    //motors/encoders
      // intakeBottomMotor = new CANSparkMax(MotorIDConstants.intakeBottomMotorID, MotorType.kBrushless);
      intakeTop1Motor = new CANSparkMax(MotorIDConstants.intakeTop1MotorID, MotorType.kBrushless);
      intakeTop2Motor = new CANSparkMax(MotorIDConstants.intakeTop2MotorID, MotorType.kBrushless);
      flipoutRun = new TalonFX(MotorIDConstants.intakeRunExtended);
      beamBreak = new DigitalInput(SensorConstants.intakeBeamBreakDIOPort);

      intakeTop2Motor.follow(intakeTop1Motor, true);    

      // intakeBottomMotor.setOpenLoopRampRate(0.4);
      intakeTop1Motor.setOpenLoopRampRate(0.4);
      intakeTop2Motor.setOpenLoopRampRate(0.4);
      
  }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putBoolean("beam break tripped", !beamBreak.get());
  }

  public void stop(){
    // intakeBottomMotor.set(0);
    intakeTop1Motor.set(0);
    flipoutRun.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void intake(){
    // intakeBottomMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed);
    flipoutRun.set(TalonFXControlMode.PercentOutput, MotorSpeedsConstants.flipOutRunSpeed);
  }

  public void intakeWithoutFlipout(){
    // intakeBottomMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed);  }

  public void feed(){
    // intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoFeedSpeed);
  }

  public void setOut(){
    // intakeBottomMotor.set(-MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void stopBottom(){
    // intakeBottomMotor.set(0);
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed);
  }

  public void manual(CommandXboxController controller){
    intakeTop1Motor.set(MotorSpeedsConstants.intakeNeoSpeed*controller.getHID().getRawAxis(ControllerConstants.intakeTopAxis));
    // intakeBottomMotor.set(MotorSpeedsConstants.intakeNeoSpeed*-controller.getHID().getRawAxis(ControllerConstants.intakeBottomAxis));
  }

  public boolean getBBTripped(){
    return !beamBreak.get();
  }

  public void stopFlipoutRun(){
    flipoutRun.set(TalonFXControlMode.PercentOutput, 0);
  }

}
