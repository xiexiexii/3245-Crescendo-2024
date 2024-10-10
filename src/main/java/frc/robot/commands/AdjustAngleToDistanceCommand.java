package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TeleopTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.utils.ShoulderRegression;

public class AdjustAngleToDistanceCommand extends Command{
    //instantiate stuff
    ShoulderSubsystem m_shoulderSubsystem;

    public AdjustAngleToDistanceCommand(ShoulderSubsystem shoulderSubsystem){
        //definitions and setting parameters equal to members
        m_shoulderSubsystem = shoulderSubsystem;
        addRequirements(m_shoulderSubsystem);
    }

    @Override
    public void initialize() {
    }
        
    @Override
    public void execute() {
        m_shoulderSubsystem.setDistShot();
    }

    @Override
    public void end(boolean interrupted){
       
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}