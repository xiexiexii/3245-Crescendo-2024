package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeIntoShooterCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    public IntakeIntoShooterCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {
        m_intakeSubsystem.stopBottom();
        m_indexerSubsystem.runSlow();
    }

    @Override
    public void end(boolean interrupted){
        m_indexerSubsystem.stop();
        m_intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.getBBTripped();
    }

}