package org.firstinspires.ftc.teamcode;

//Dpad Up = speed 100%
//Dpad Down = speed 50%
//RT Raises intake
//LT Lowers intake
//Second player can also control intake with left stick for fine control, or RT/LT
//Button A does intake (toggle)
//Button X is reverse intake (toggle)
//Button Y is slow intake (50% speed) (toggle)
//Dpad left is actuator down (hold)
//Dpad right is actuator up (hold)

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//*In theory* this should also be compatible with tank drive, except for the strafing parts

@TeleOp(name = "Main TeleOp (tap me)", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    public static final double SLIDE_SPEED = 0.95;
    public static final double INTAKE_SPEED = 0.92;

    /* Declare OpMode members. */
    TeleOpMap robot = new TeleOpMap();   //Configs hardware


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        //loads hardwareMap

        double drive;
        double turn;
        double strafe;
        double leftValue;
        double rightValue;
        double powerFL;
        double powerFR;
        double powerRL;
        double powerRR;
        double slidePower;
        boolean runintake = false;
        boolean reverseintake = false;
        boolean slowintake = false;
        double speed = 0.5;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Shock drone going live!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "ASSUMING DIRECT CONTROL");
        telemetry.update();

        while (opModeIsActive()) {
            //speed is
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                speed = 1;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                speed = 0.5;
            }
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;


            // Combine drive and turn for blended motion.
            leftValue = drive + turn;
            rightValue = drive - turn;
            powerFL = leftValue + strafe;
            powerFR = rightValue - strafe;
            powerRL = leftValue - strafe;
            powerRR = rightValue + strafe;

            //applies acceleration curve
            powerFL *= Math.abs(powerFL);
            powerFR *= Math.abs(powerFR);
            powerRL *= Math.abs(powerRL);
            powerRR *= Math.abs(powerRR);

            //applies speed limiter
            powerFL *= speed;
            powerFR *= speed;
            powerRL *= speed;
            powerRR *= speed;

            //makes sure motor values aren't insane
            powerFL = Range.clip(powerFL, -speed, speed);
            powerFR = Range.clip(powerFR, -speed, speed);
            powerRL = Range.clip(powerRL, -speed, speed);
            powerRR = Range.clip(powerRR, -speed, speed);

            //sets motor power
            robot.motorFL.setPower(powerFL);
            robot.motorFR.setPower(powerFR);
            robot.motorRL.setPower(powerRL);
            robot.motorRR.setPower(powerRR);

            //right trigger raises, left trigger lowers
            //THIS WAS OPPOSITE DURING THE MATCH, DOUBLE CHECK THIS

            //both gamepads can control the arm
            //gamepad2 can use left stick for fine arm control
            slidePower = (((gamepad1.left_trigger + gamepad2.left_trigger) + (-gamepad2.left_stick_y)) - (gamepad1.right_trigger + gamepad2.right_trigger));
            slidePower *= SLIDE_SPEED;

            //sets maxes for each value
            slidePower = Range.clip(slidePower, -SLIDE_SPEED, SLIDE_SPEED);

            robot.slide.setPower(slidePower);

            if (gamepad1.a || gamepad2.a) {
                runintake = true;
                reverseintake = false;
                slowintake = false;
            }
            if (gamepad1.b || gamepad2.b) {
                runintake = false;
                reverseintake = true;
                slowintake = false;
            }
            if (gamepad1.x || gamepad2.x) {
                runintake = false;
                reverseintake = false;
                slowintake = true;
            }
            if (gamepad1.y || gamepad2.y) {
                runintake = false;
                reverseintake = false;
                slowintake = false;
            }

            if (runintake) {
                robot.intake.setPower(INTAKE_SPEED);
            } else if (reverseintake) {
                robot.intake.setPower(-0.125);
            } else if (slowintake) {
                robot.intake.setPower(0.5);
            } else {
                robot.intake.setPower(0);
            }

            if (gamepad1.dpad_left || gamepad2.dpad_down || gamepad2.dpad_left) {
                robot.actuator.setPower(-1);
            } else if (gamepad1.dpad_right || gamepad2.dpad_up || gamepad2.dpad_right) {
                robot.actuator.setPower(1);
            } else {
                robot.actuator.setPower(0);
            }

            telemetry.addData("Status", "Speed: " + speed + "\n" +
                    "Power: " + drive + "        Turn: " + turn + "        Strafe: " + strafe + "\n" +
                    "Slide Power: " + slidePower + "     intake Power: " + INTAKE_SPEED);
            telemetry.update();
        }
    }
    //idle();
}