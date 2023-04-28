package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WaitCommandTBD extends WaitCommand
{
    Telemetry telemetry;


    public WaitCommandTBD(long milliseconds, Telemetry telem)
    {
        super(milliseconds);
        telemetry = telem;
    }

    @Override
    public void execute()
    {
        telemetry.addData( "Waiting: ", "%.2f seconds.", m_timer.elapsedTime()/1000.0);
        super.execute();
    }
}
