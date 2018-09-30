package org.firstinspires.ftc.teamcode;

/*
 * Created by on 9/24/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;



/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="HolonomicTeleOp", group="Pushbot")
public class HolonomicTeleOP_POV extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareRobot robot           = new HardwareRobot();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
double counter;
    @Override
    public void runOpMode() {
        double ch1;
        double ch2;
        double ch3;
        double FLvalue;
        double FRvalue;
        double BLvalue;
        double BRvalue;
        boolean pinionup;
        boolean piniondown;
        double          clawOffset      = 0;
        final double    CLAW_SPEED      = 0.02 ;


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Kitty");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            ch1=gamepad1.right_stick_x ;
            ch3=gamepad1.left_stick_x ;
            ch2=gamepad1.left_stick_y ;
            pinionup = gamepad1.dpad_up;
            piniondown = gamepad1.dpad_down;

            telemetry.addData("say" , pinionup);
            telemetry.update();



            ch1 = Range.clip(ch1, -1, 1);
            ch2 = Range.clip(ch2, -1, 1);
            ch3 = Range.clip(ch3, -1, 1);





            // Combine drive and turn for blended motion.
            FLvalue =  + ch2 - ch3 + ch1;
            FRvalue =  - ch2 - ch3 + ch1;
            BLvalue =  + ch2 + ch3 + ch1;
            BRvalue =  - ch2 + ch3 + ch1;
            //Normalize the values so neither exceed +/- 1.0
            //max = Math.max(Math.abs(left), Math.abs(right));
            /*if (max > 1.0)
            {
                left /= max;
                right /= max;
            }
            */

            // Output the safe vales to the motor drives.
            robot.frontRight.setPower(FRvalue);
            robot.frontLeft.setPower(FLvalue);
            robot.backLeft.setPower(BLvalue);
            robot.backRight.setPower(BRvalue);
            if(piniondown){
                robot.rackpinion.setPower(1.0);
                counter++;

            }
            else if(pinionup){
                robot.rackpinion.setPower(-1.0);
                counter--;
            }
            else{
                robot.rackpinion.setPower(0);
            }


            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1 .left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);

            robot.upleftClaw.setPosition(robot.MID_SERVO + clawOffset);

            robot.uprightClaw.setPosition(robot.MID_SERVO - clawOffset);



            // Use gamepad buttons to move arm up (Y) and down (A)
            /*if (gamepad1.right_bumper)
                clawOffset  += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            robot.scissorclaw1.setPosition(robot.MID_SERVO + clawOffset);
            robot.scissorclaw2.setPosition(robot.MID_SERVO - clawOffset);
            */

            telemetry.addData("Say", counter);
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}




