package org.firstinspires.ftc.teamcode.LastSeason.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ArmSubsystem;

public class WristCommand extends CommandBase
{
    private final double myAngle;
    private final ArmSubsystem myArm;


    public WristCommand(double angle, ArmSubsystem arm )
    {
        myAngle = angle;
        myArm = arm;
        addRequirements(myArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        myArm.wristMove(myAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
