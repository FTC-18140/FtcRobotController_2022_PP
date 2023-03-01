// Makes all code compatible with the FTC variables
package org.firstinspires.ftc.teamcode;

// Specific Libraries of commands to import from the package
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * The type Teleop.
 */
// Initializes the main framework for the program
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode
{

    /**
     * The Robot.
     */
// Calls new Robot and gives it a name
    Thunderbot_2022 robot = new Thunderbot_2022();

    /**
     * The Wrist position.
     */
// Variables for the positions for claw, wrist, and elbow
    double wristPosition = 0;
    /**
     * The Claw position.
     */
    double clawPosition = 0;
    /**
     * The Elbow position.
     */
    double elbowPosition = 0.535;

    /**
     * The Elbow increment.
     */
    double ELBOW_INCREMENT = 0.005;
    /**
     * The Wrist increment.
     */
    double WRIST_INCREMENT = 0.015; // its 0.0025
    /**
     * The Claw increment.
     */
    double CLAW_INCREMENT = 0.0125;

    // All the things that happen when the init button is pressed
    @Override
    public void init() {
        telemetry.addData("Init", "Start");

        // Calls and initializes all values and parts of the robot declared in ThunderBot_2022
        robot.init(hardwareMap, telemetry, false);

        // Displays Init: Done
        telemetry.addData("Init", "Done");

        // Makes sure the Claw and Wrist can move as free as they need to
        try {
            wristPosition = 0.55;
            clawPosition = 0.3;
            robot.armstrong.elbowMove(elbowPosition);
            robot.armstrong.wristMove(wristPosition);
            robot.armstrong.clawMove(clawPosition);
        } catch (Exception e) {
            telemetry.addData("cant", "run");
        }

    }

    public void init_loop() {
        telemetry.addData("Lelbow Position", robot.armstrong.leftElbow.getPosition());
        telemetry.addData("Relbow Position", robot.armstrong.rightElbow.getPosition());
        telemetry.addData("Wrist Position", robot.armstrong.wrist.getPosition());
        telemetry.addData("Claw Position: ", robot.armstrong.claw.getPosition());
    }

    // All the things it does when you select Play button
    @Override
    public void start() {
        robot.start();
    }

    // All the things it does over and over during the period from when start is pressed to when stop is pressed.
    @Override
    public void loop() {
        robot.update();

        /////////////////
        // CHASSIS
        /////////////////
        if (gamepad1.a) {
            // TURBO!!!
            robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else if (gamepad1.x){
            // 20 percent
            robot.joystickDrive(-gamepad1.left_stick_y * 0.2, gamepad1.left_stick_x * 0.2, gamepad1.right_stick_x * 0.2);
        } else {
            // Normal Drive
            double sign = Math.signum(gamepad1.right_stick_x);
            robot.joystickDrive(-gamepad1.left_stick_y * 0.6, gamepad1.left_stick_x * 0.6, gamepad1.right_stick_x * 0.8 * gamepad1.right_stick_x * sign);
        }

//        /////////////////
//        // TWIST
//        /////////////////
//
//        // TODO: Add manual control/tweaking of twist here.  The automatic twist is now being handled
//        // by the Armstrong class.
//        //
//        if (gamepad2.dpad_left) {
//            robot.armstrong.armRotate(1);
//        } else if (gamepad2.dpad_right) {
//            robot.armstrong.armRotate(0);
//        }

        /////////////////
        // CLAW
        /////////////////
        if (gamepad2.left_bumper) {
            clawPosition += CLAW_INCREMENT;
        } else if (gamepad2.right_bumper) {
            clawPosition -= CLAW_INCREMENT;
        }
        clawPosition = Range.clip(clawPosition, robot.armstrong.getCLAW_MIN(), robot.armstrong.getCLAW_MAX());
        robot.armstrong.clawMove(clawPosition);


        /////////////////
        // ELBOW
        /////////////////
        if (gamepad2.b) {
            elbowPosition -= ELBOW_INCREMENT;
        } else if (gamepad2.x) {
            elbowPosition += ELBOW_INCREMENT;
        }

        elbowPosition = Range.clip(elbowPosition, robot.armstrong.getELB_MIN(), robot.armstrong.getELB_MAX());
        robot.armstrong.elbowMove(elbowPosition);

        /////////////////
        // LINEAR SLIDE
        /////////////////
        if (gamepad2.left_stick_button) {
            if (gamepad2.y) {
                robot.armstrong.liftFree(0.25);
            } else if (gamepad2.a) {
                robot.armstrong.liftFree(-0.25);
            } else {
                robot.armstrong.liftStop();
            }
        } else {
            if (gamepad2.y) {
                robot.armstrong.normalGoUp();
            }

            if (gamepad2.a) {
                robot.armstrong.normalGoDown();
            }
        }
        
        /////////////////
        // RESET
        /////////////////
        if (gamepad2.left_trigger > 0.9 && gamepad2.right_trigger > 0.9) {
            robot.armstrong.resetEncoders();
            robot.armstrong.leftSlidePosition = 0;
            robot.armstrong.rightSlidePosition = 0;
            telemetry.addData("resetting left encoders.", 0);
        }

        /////////////////
        // WRIST
        /////////////////
        if(gamepad2.dpad_up) {
            wristPosition += WRIST_INCREMENT;
            wristPosition = Range.clip(wristPosition, robot.armstrong.getWRIST_MIN(), robot.armstrong.getWRIST_MAX());
            robot.armstrong.wristMove(wristPosition);
        } else if (gamepad2.dpad_down) {
            wristPosition -= WRIST_INCREMENT;
            wristPosition = Range.clip(wristPosition, robot.armstrong.getWRIST_MIN(), robot.armstrong.getWRIST_MAX());
            robot.armstrong.wristMove(wristPosition);
        }
        telemetry.addData("Wrist Position", robot.armstrong.wrist.getPosition());


        // presets for putting the cones at the exact height needed to score

        ////////////
        // Mid forward
        ////////////

        if (gamepad2.dpad_left)
        {
            robot.armstrong.wristMove(0.135);
            wristPosition = robot.armstrong.getWristPosition();

            robot.armstrong.elbowMove(0.3475);
            elbowPosition = robot.armstrong.getElbowPosition();

            robot.armstrong.midForwardLift();
        }

        ////////////
        // Mid backward
        ////////////
        if (gamepad2.dpad_right)
        {
            robot.armstrong.wristMove(0.18);
            wristPosition = robot.armstrong.getWristPosition();

            robot.armstrong.elbowMove(0.28);
            elbowPosition = robot.armstrong.getElbowPosition();

            robot.armstrong.midBackwardLift();
        }

        ////////////
        // High forward
        ////////////
        if (gamepad2.left_stick_button) {
            if (gamepad2.x) {
                robot.armstrong.wristMove(0.08);
                wristPosition = robot.armstrong.getWristPosition();

                robot.armstrong.elbowMove(0.34);
                elbowPosition = robot.armstrong.getElbowPosition();

                robot.armstrong.highForwardLift();
            }
        }

        ////////////////////
        // High backward
        ////////////////////
        if (gamepad2.left_stick_button) {
            if (gamepad2.b) {
                robot.armstrong.wristMove(0.07);
                wristPosition = robot.armstrong.getWristPosition();

                robot.armstrong.elbowMove(0.285);
                elbowPosition = robot.armstrong.getElbowPosition();

               robot.armstrong.highBackwardLift();
            }
        }

        /////////////////
        // Sensors
        /////////////////

//        robot.armstrong.detectColor();
//        if (robot.armstrong.color.red() > 200 && robot.armstrong.color.green() > 200
//                && robot.armstrong.color.red() < 400 && robot.armstrong.color.green() < 400) {
//            telemetry.addData("yellow", "is detected");
//        } else {
//            telemetry.addData("yellow", "is not detected");
//        }

       // robot.armstrong.detectDistance();
    }
}
