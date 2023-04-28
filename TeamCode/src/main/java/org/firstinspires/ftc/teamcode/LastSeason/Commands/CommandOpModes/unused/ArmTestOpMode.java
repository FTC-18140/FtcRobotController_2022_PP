package org.firstinspires.ftc.teamcode.LastSeason.Commands.CommandOpModes.unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.LastSeason.Commands.CommandOpModes.TBDOpModeBase;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.ClawCommand;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.ElbowCommand;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.WaitCommandTBD;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.WristCommand;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.LastSeason.Commands.Subsystems.ClawSubsystem;

@Autonomous(name = "FTCLib_ElbowOpMode", group = "FTCLib")
@Disabled
public class ArmTestOpMode extends TBDOpModeBase
{
    ArmSubsystem arm = null;
    ClawSubsystem claw = null;

    @Override
    public void init() {
        try {

            arm = new ArmSubsystem(hardwareMap, telemetry);
            claw = new ClawSubsystem(hardwareMap, telemetry);
            ElbowCommand goUp = new ElbowCommand(0.275, arm);
            ElbowCommand goDown = new ElbowCommand(0.535, arm);
            WaitCommandTBD waitTime = new WaitCommandTBD(5000, telemetry);
            WristCommand moveUp = new WristCommand(0.135, arm);
            WristCommand moveDown = new WristCommand(0.155, arm);
            ClawCommand openUp = new ClawCommand(0.525, claw, "none");
//            claw.clawMove(0.3);
            register(claw);
            register(arm);


//            SequentialCommandGroup ElbowMotion = new SequentialCommandGroup(goUp, moveUp, openUp, waitTime, goDown);
//            schedule(ElbowMotion);
            schedule(openUp);

        } catch (Exception e) {
            telemetry.addData("Something did not initialize properly.", 0);
            telemetry.addData("Ugh: ", e.getMessage());
            telemetry.addData("Ugh2: ", "%s\n%s\n%s\n%s\n%s", e.getStackTrace()[0], e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3], e.getStackTrace()[4]);
        }
    }

    @Override
    public void init_loop() {
        //if (odometry != null) { odometry.update(); }
        //telemetry.addData("Init Loop is running... ", 1);
    }
}