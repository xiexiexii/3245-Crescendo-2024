package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class IntakeIndexUntilTrippedCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    IndexerSubsystem m_indexerSubsystem;
    ShootSubsystem m_shootSubsystem;
    public IntakeIndexUntilTrippedCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, 
            ShootSubsystem shootSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_indexerSubsystem = indexerSubsystem;
        m_shootSubsystem = shootSubsystem;
        addRequirements(m_intakeSubsystem);
        addRequirements(m_indexerSubsystem);
        addRequirements(m_shootSubsystem);
    }

    @Override
    public void initialize() {

    }
        
    @Override
    public void execute() {
        m_intakeSubsystem.intake();
        m_indexerSubsystem.runFast();
        m_shootSubsystem.driveBackward();
    }

    @Override
    public void end(boolean interrupted){
        m_intakeSubsystem.stop();
        m_indexerSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.getBBTripped();
    }

}