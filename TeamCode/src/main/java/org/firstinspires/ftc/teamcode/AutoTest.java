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
       // TODO: add code to get the signal sleeve value from the Eyes class.
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
                    done = robot.gyroDrive(0, 45, 0.2);
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
                    done = robot.turn(-45, -0.2);
                    telemetry.addData("case 1", "is started");
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
              //      done = robot.gyroDrive(90, 0, 0.2);
                    telemetry.addData("case 2", "is started");
                    done = true;
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
                    done = robot.gyroDrive(-45, 8, -0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 4:
                if (!done) {
                    done = robot.turn(45, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
            case 5:
                if (!done) {
                    done = robot.drive(90, 41, 0.2);
                } else {
                    robot.stop();
                    done = false;
                    telemetry.addData("program", "ended");
                    state++;
                }
                break;
            default:
                break;
        }
    }
}
