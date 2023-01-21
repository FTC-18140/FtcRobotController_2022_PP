package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

public class ElbowCommand extends CommandBase
{
    private final double myAngle;
    private final ArmSubsystem theSubsystem;

    ElbowCommand(double angle, ArmSubsystem arm )
    {
        myAngle = angle;
        theSubsystem = arm;
    }

    @Override
    public void initialize() {
        theSubsystem.elbowMove(myAngle);
    }

}
