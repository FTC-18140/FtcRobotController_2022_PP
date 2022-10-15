package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class AutoTest extends OpMode
{
    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    int state = 0;

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop()
    {
       // TODO: add code to get the signal sleeve value
    }

    @Override
    public void start()
    {
        telemetry.addData("Robot Location: ", robot.updateHeading());
    }

    @Override
    public void loop()
    {
        switch (state)
        {
            case 0:
                if (!done)
                {
                    done = robot.drive(0, 40, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 1:
                if (!done)
                {
                    done = robot.turnTo(90, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 2:
                if (!done)
                {
                    done = robot.gyroDrive(90, 40, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 3:
                if (!done)
                {
                    done = robot.turnTo(0, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            default:
                break;
        }
    }
}
