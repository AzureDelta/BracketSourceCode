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

@TeleOp(name = "Drivetrain", group = "TeleOp")
public class DrivetrainTest extends LinearOpMode {

    public static final double ARM_SPEED = 0.90;
    public static final double INTAKE_SPEED = 0.80;

    /* Declare OpMode members. */
    TeleOpMap robot = new TeleOpMap();   //Configs hardware

    //NeverRest 40 motor: Diameter = 0.85 inches
    //NeverREST: 120 rpm
    //Circumference = 2.669 inches
    //NeverRest: IPM: 320.28 inches/min
    //GoBilda Yellow Jacket motor: Diameter = 1 inch
    //GoBilda: 84 rpm
    //Circumference: 3.14 inches
    //GoBilda: IPM: 263.76 inches/min

    //To run at the same rate, the NeverRest must run at 82.35% of it's speed
    //This puts both motors running @ approx. 263.75 in/min

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
        int counterUpTighten = 0;
        int counterUpLoosen = 0;
        int counterDownTighten = 0;
        int counterDownLoosen = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Shock drone going live!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "ASSUMING DIRECT CONTROL");
        telemetry.update();

        while (opModeIsActive()) {
            //speed is
            if (gamepad1.left_stick_button) {
                speed = 1;
            }
            if (gamepad1.right_stick_button) {
                speed = 0.5;
            }
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            // Combine drive and turn for blended motion.
            leftValue = drive - turn;
            rightValue = drive + turn;
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



            telemetry.addData("Status", "Speed: " + speed + "\n" +
                    "Power: " + drive + "        Turn: " + turn + "        Strafe: " + strafe + "\n" +
                    "Slide Power: " + "     intake Power: " + INTAKE_SPEED + "\n" +
                    "Counter Up Tighten: " + counterUpTighten + "Counter Down Tighten: " + counterDownTighten + "\n"
             + "Counter Up Loosen: " + counterUpLoosen + "Counter Down Loosen: " + counterDownLoosen);
            telemetry.update();
        }
    }
    //idle();
}