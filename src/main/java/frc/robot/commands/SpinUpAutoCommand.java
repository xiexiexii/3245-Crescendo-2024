package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SpinUpAutoCommand extends Command{
    //instantiate stuff
    ShootSubsystem m_shootSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    Timer timer = new Timer();

    public SpinUpAutoCommand(ShootSubsystem shootSubsystem){
        //definitions and setting parameters equal to members
        m_shootSubsystem = shootSubsystem;
        addRequirements(m_shootSubsystem);
        }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }
        
    @Override
    public void execute() {
        if(timer.get()<AutoTimeConstants.spinUpAutoTime2){
            m_shootSubsystem.spinUpAuto();
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return timer.get()>AutoTimeConstants.spinUpAutoTime2;
    }

}