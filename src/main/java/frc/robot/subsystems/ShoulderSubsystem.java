// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.PositionValueConstants;
import frc.robot.utils.ShoulderRegression;


public class ShoulderSubsystem extends SubsystemBase {
    //init stuff
      TalonFX shoulderMaster;
      TalonFX shoulderFollower;
      CANcoder canCoder; 
      TalonFXConfiguration falconConfig;
      Slot0Configs slot0Configs;
      CANcoderConfiguration canCoderConfig;
      PositionDutyCycle positionDutyCycle;
      VoltageOut voltage;
    
  
  public ShoulderSubsystem() {
    //motors/encoders/pidcontroller
      shoulderFollower = new TalonFX(MotorIDConstants.shoulder2MotorID);
      shoulderMaster = new TalonFX(MotorIDConstants.shoulder3MotorID);

    //CANCoder 
      falconConfig = new TalonFXConfiguration();
      canCoder = new CANcoder(MotorIDConstants.shoulderCANCoderID);
      falconConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
      falconConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    //other falcon configs
      falconConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = MotorSpeedsConstants.shoulderRampRate;
      falconConfig.MotorOutput.PeakForwardDutyCycle = MotorSpeedsConstants.shoulderClosedMaxSpeed;
      falconConfig.MotorOutput.PeakReverseDutyCycle = -MotorSpeedsConstants.shoulderClosedMaxSpeed;
      falconConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      falconConfig.CurrentLimits.SupplyCurrentLimit = 40;

    //pid
      slot0Configs = new Slot0Configs();
      slot0Configs.kP = PIDConstants.shoulderkP;
      slot0Configs.kI = PIDConstants.shoulderkI;
      slot0Configs.kD = PIDConstants.shoulderkD;

    //apply configs, inversion, control requests
      shoulderMaster.getConfigurator().apply(falconConfig);
      shoulderFollower.getConfigurator().apply(falconConfig);
      shoulderMaster.getConfigurator().apply(slot0Configs, 0.05);

      shoulderMaster.setInverted(false);
      shoulderFollower.setControl(new Follower(shoulderMaster.getDeviceID(), true));

      positionDutyCycle = new PositionDutyCycle(0, 0, false, 0, 0, 
        false, false, false);
      
      voltage = new VoltageOut(0, false, false, false, false);

      }

  @Override
  public void periodic() {
    //smartdashboard shenanigans
    SmartDashboard.putNumber("Shoulder Encoder Value:", canCoder.getPosition().getValueAsDouble());
    }

  public void resetEncoder(){
    canCoder.setPosition(0);
  }

  public void setHome(){
    shoulderMaster.setControl(positionDutyCycle.withPosition(PositionValueConstants.shoulderHomePos));
    SmartDashboard.putBoolean("Shoulder At Amp Height", false);
  }

  public void setAmpShot(){
    shoulderMaster.setControl(positionDutyCycle.withPosition(PositionValueConstants.shoulderAmpShotPos));
    SmartDashboard.putBoolean("Shoulder At Amp Height", true);
  }

  public void setProtShot(){
    shoulderMaster.setControl(positionDutyCycle.withPosition(PositionValueConstants.shoulderProtShotPos));
    SmartDashboard.putBoolean("Shoulder At Amp Height", false);
  }

  public void setDistShot(){
    shoulderMaster.setControl(positionDutyCycle.withPosition(
      ShoulderRegression.distanceToShoulderCounts(SmartDashboard.getNumber("Distance to target (meters)", 1.252))));
      SmartDashboard.putBoolean("Shoulder At Amp Height", false);
  }

  public void bumpUp(){
    shoulderMaster.setControl(positionDutyCycle.withPosition(
      canCoder.getPosition().getValueAsDouble() + 0.1));
  }

  public void bumpDown(){
    shoulderMaster.setControl(positionDutyCycle.withPosition(
      canCoder.getPosition().getValueAsDouble() - 0.001));
  }

  public void manual(CommandXboxController controller){
    shoulderMaster.setControl(voltage.withOutput(12*0.3*controller.getHID().getRawAxis(ControllerConstants.shoulderAxis)));
  }
 
}
