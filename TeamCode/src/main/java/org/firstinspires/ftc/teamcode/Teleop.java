// Makes all code compatible with the FTC variables
package org.firstinspires.ftc.teamcode;

// Specific Libraries of commands to import from the package
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

// Initializes the main framework for the program
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {

    // Calls new Robot and gives it a name
    Thunderbot_2022 robot = new Thunderbot_2022();

    // Calls two new variables for the positions for claw/wrist
    double wristPosition = 0;
    double clawPosition = 0;
    // Defining the Variables that will be used for the steps in the autonomous

    // All the things that happen when the init button is pressed
    @Override
    public void init() {
        // Displays Init: Start
        telemetry.addData("Init", "Start");

        // Calls and initalizes all values and parts of the robot declared in ThunderBot_2022
        robot.init(hardwareMap, telemetry);

        // Displays Init: Done
        telemetry.addData("Init", "Done");

        // Makes sure the Claw and Wrist can move as free as they need to
        try {
            wristPosition = 0.625;
            clawPosition = 1;
            robot.slide.wristMove(wristPosition);
            robot.slide.clawMove(clawPosition);
        } catch (Exception e) {
            telemetry.addData("cant", "run");
        }

    }

    // All the things it does when you select Play button
    @Override
    // Displays Starting:...
    public void start() {
        telemetry.addData("Starting", "...");
    }

    // All the things it does over and over during the period from when start is pressed to when stop is pressed.
    @Override
    public void loop() {

        // Makes it so when Left stick isn't pressed, speed is at half, but when it is pressed it's at full speed
        if (gamepad1.a) {
            robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        } else {
            robot.joystickDrive(-gamepad1.left_stick_y * 0.4, gamepad1.left_stick_x * 0.4, -gamepad1.right_stick_x * 0.4);
        }

        // Tells what values are being inputted into the joystick.
        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        telemetry.addData("Magnet Sensor is", robot.slide.limit.isPressed());
        // Shows degree value or position of the wrist/claw
        try {
            telemetry.addData("Wrist Position", robot.slide.wrist.getPosition());
            telemetry.addData("Claw Position", robot.slide.claw.getPosition());
        // If something's wrong and it can't display it, then it displays this
        } catch (Exception e) {
            telemetry.addData("cant show", "telemetry");
        }

        // Calls to bend the wrist up and down
        if (gamepad2.dpad_up) {
            wristPosition += 0.0025;
        } else if (gamepad2.dpad_down) {
            wristPosition -= 0.0025;
        }

        // Sets the range of motion for the wrist servo
        wristPosition = Range.clip(wristPosition, 0, 0.625);
        robot.slide.wristMove(wristPosition); // corrects position

        // Sets the input of how much the claw moves with each button press/hold
        if (gamepad2.left_bumper) {
            clawPosition += 0.003;
        } else if (gamepad2.right_bumper) {
            clawPosition -= 0.003;// this value was 0.0025 before
        }

        // Sets the range of motion for the claw servo
        clawPosition = Range.clip(clawPosition, 0.2, 1);
        robot.slide.clawMove(clawPosition);

        // Maps buttons for the linear slide up/down
        if (gamepad2.y) {
            robot.slide.reverse();
        } else if (gamepad2.a) {
            robot.slide.extend();
        } else {
            robot.slide.stopExtend();
        }

//        if (gamepad1.right_bumper) {
//            robot.slide.close();
//        } else if (gamepad1.left_bumper) {
//            robot.slide.clawMove();
//        }

        // Maps button for clockwise/counterclockwise of core hex motor for arm
        if (gamepad2.x) {
            robot.slide.liftArm();
        } else if (gamepad2.b) {
            robot.slide.lowerArm();

            // If nothing's being pressed, then don't move
        } else {
            robot.slide.armStop();
        }
    }
}
