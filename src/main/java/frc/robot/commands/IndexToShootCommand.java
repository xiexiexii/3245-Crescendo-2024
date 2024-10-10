package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoTimeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class IndexToShootCommand extends Command{
    //instantiate stuff
    ShootSubsystem m_shootSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    Timer timer = new Timer();

    public IndexToShootCommand(ShootSubsystem shootSubsystem, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem){
        //definitions and setting parameters equal to members
        m_shootSubsystem = shootSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_shootSubsystem);
        addRequirements(m_indexerSubsystem);
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }
        
    @Override
    public void execute() {
        if(timer.get()<AutoTimeConstants.indexAutoTime2){
            m_indexerSubsystem.runFast();
            m_intakeSubsystem.intake();
        }
        if (timer.get()>AutoTimeConstants.indexAutoTime2){
            m_indexerSubsystem.stop();
            m_shootSubsystem.stop();
            m_intakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_shootSubsystem.stop();
        m_intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return timer.get()>AutoTimeConstants.indexAutoTime2;
    }

}