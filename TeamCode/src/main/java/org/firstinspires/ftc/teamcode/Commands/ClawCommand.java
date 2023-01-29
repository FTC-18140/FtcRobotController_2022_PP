package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase
{
    private final double myAngle;
    private final ClawSubsystem myClaw;

    ClawCommand(double angle, ClawSubsystem claw )
    {
        myAngle = angle;
        myClaw = claw;
        addRequirements(myClaw);
    }

    @Override
    public void initialize() {
        myClaw.clawMove(myAngle);
    }

}
