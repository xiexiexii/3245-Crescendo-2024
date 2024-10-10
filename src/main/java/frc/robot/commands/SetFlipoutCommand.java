package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeFlipoutSubsystem;

public class SetFlipoutCommand extends Command{
    //instantiate stuff
    IntakeFlipoutSubsystem m_flipoutSubsystem;;
    String m_angle;

    public SetFlipoutCommand(IntakeFlipoutSubsystem flipoutSubsystem, String angle){
        //definitions and setting parameters equal to members
        m_flipoutSubsystem = flipoutSubsystem;
        m_angle = angle;
        addRequirements(m_flipoutSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {

        if(m_angle == "in"){
            m_flipoutSubsystem.setIn();
        }

        if(m_angle == "out"){
            m_flipoutSubsystem.setOut();
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }

}