package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class RiverTest1 extends OpMode {
    Thunderbot_2022 robot = new Thunderbot_2022();
    boolean done = false;
    double power = 0.4;

    int step = 0;



    double[] steps = {
            20,
            -45,
            20,
            0.4,
            -20,
            45,
            1
    };

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop(){}

    @Override
    public void start() {
        telemetry.addData("Robot Location: ", robot.updateHeading());
    }

    @Override
    public void loop()
    {
        if(step >= steps.length-1){
            robot.stop();
        }else {
            switch (step) {
                case (0):
                    if(!done){
                        done = robot.drive(0, steps[step], power) && robot.armstrong.clawMove(steps[6]);
                    }else{
                        done = false;
                        step++;
                    }
                    break;
                case (1):
                    if(!done){
                        done = robot.turnTo(steps[step], power);
                    }else{
                        done = false;
                        step++;
                    }

                    break;
                case (2):
                    if(!done){
                        done = robot.drive(0, steps[step], power);
                    }else{
                        done = false;
                        step++;
                    }

                    break;
                case (3):
                    if(!done){
                        done = robot.armstrong.clawMove(steps[step]);
                    }else{

                        robot.stop();
                        done = false;
                        step++;
                    }

                    break;
                case (4):
                    if(!done){
                        done = robot.drive(0, steps[step], power);
                    }else{
                        done = false;
                        step++;
                    }

                    break;
                case (5):
                    if(!done){
                        done = robot.turnTo(steps[step], power);
                    }else{
                        done = false;
                        step++;
                    }

                    break;
                case (6):
                    if(!done){
                        done = robot.armstrong.clawMove(steps[step]);
                    }else{

                        robot.stop();
                        done = false;
                        step++;
                    }

                    break;
                default:
                    robot.stop();
                    break;
            }
        }

    }


}
