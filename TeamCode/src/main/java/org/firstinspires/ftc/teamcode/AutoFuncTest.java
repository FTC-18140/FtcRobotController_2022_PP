package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class AutoFuncTest extends OpMode
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
                    done = robot.gyroDrive(0, 20, 0.2);
                }
                else
                {
                    robot.stop();
                    done = false;
                    state++;
                }
                break;
//            case 1:
//                if (!done) {
//                    done = robot.gyroDrive(45, 20, 0.2);
//                } else {
//                    robot.stop();
//                    done = false;
//                    state++;
//                }
//                break;
//            case 2:
//                if (!done) {
//                    done = robot.gyroDrive(90, 20, 0.2);
//                } else {
//                    robot.stop();
//                    done = false;
//                    state++;
//                }
//                break;
            default:
                break;
        }
    }
}
