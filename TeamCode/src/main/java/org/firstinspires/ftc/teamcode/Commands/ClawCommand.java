package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase
{
    private final double myAngle;
    private final ClawSubsystem theSubsystem;

    ClawCommand(double angle, ClawSubsystem claw )
    {
        myAngle = angle;
        theSubsystem = claw;
    }

    @Override
    public void initialize() {
        theSubsystem.clawMove(myAngle);
    }

}
