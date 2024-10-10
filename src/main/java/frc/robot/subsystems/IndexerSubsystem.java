// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedsConstants;
import frc.robot.Constants.SensorConstants;

public class IndexerSubsystem extends SubsystemBase {
    //init stuff
      CANSparkMax indexerMotor;
      DigitalInput beamBreak;

  public IndexerSubsystem() {
    //motor
       indexerMotor = new CANSparkMax(MotorIDConstants.indexerMotorID, MotorType.kBrushless);
       beamBreak = new DigitalInput(SensorConstants.indexBeamBreakDIOPort);
  }

  @Override
  public void periodic() {}

  public void stop(){
    indexerMotor.set(0);
  }

  public void runFast(){
    indexerMotor.set(-MotorSpeedsConstants.indexRunSuckIn);
  }
  public void runSlow(){
    indexerMotor.set(MotorSpeedsConstants.indexRunMoveBack);
  }

  public void runBackSlow(){
    indexerMotor.set(MotorSpeedsConstants.indexRunMoveBack);
  }

  public boolean getBBTripped(){
    return beamBreak.get();
  }

  public void manual(CommandXboxController controller){
    if(controller.getHID().getRawButton(ControllerConstants.shootButton)){
        indexerMotor.set(-MotorSpeedsConstants.indexRunSuckIn);
    }
    else{
        indexerMotor.set(0);
    }
  }

}
