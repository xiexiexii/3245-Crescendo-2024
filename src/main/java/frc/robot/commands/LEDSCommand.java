package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSSubsystem;

public class LEDSCommand extends Command{
    //instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    LEDSSubsystem m_ledsSubsystem;
    Timer timer = new Timer();

    public LEDSCommand(IntakeSubsystem intakeSubsystem, LEDSSubsystem ledsSubsystem){
        //definitions and setting parameters equal to members
        m_intakeSubsystem = intakeSubsystem;
        m_ledsSubsystem = ledsSubsystem;
        addRequirements(m_intakeSubsystem);
        addRequirements(ledsSubsystem);
    }

    @Override
    public void initialize() {}
        
    @Override
    public void execute() {
        if(m_intakeSubsystem.getBBTripped()){
            m_ledsSubsystem.setHotPink();
        }
        else{
            m_ledsSubsystem.setTeamColor();
        }
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }

}