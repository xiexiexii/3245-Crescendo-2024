package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleForSecsCommand extends Command{
    //for S E C S ... not what you're thinking 
    //instantiate stuff
    double m_targetSecs;
    CommandXboxController m_controller;
    Timer timer = new Timer();

    public RumbleForSecsCommand(double targetSecs, CommandXboxController controller){
        //definitions and setting parameters equal to members
        m_targetSecs = targetSecs;
        m_controller = controller;
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }
        
    @Override
    public void execute() {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 1);
    }

    @Override
    public void end(boolean interrupted){
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(m_targetSecs);
    }

}