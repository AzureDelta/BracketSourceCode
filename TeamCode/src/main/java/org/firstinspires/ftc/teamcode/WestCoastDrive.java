package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="WestCoastDrive (LEGACY)", group ="TeleOp")
public class WestCoastDrive extends LinearOpMode{

    public static final double ARM_SPEED = 1;
    //public static final double ARM_SPEED = 0.5;
    public static final double INTAKE_SPEED = 0.5;

    /* Declare OpMode members. */
    HardwareConfig robot           = new HardwareConfig();   //Configs hardware


    @Override
    public void runOpMode () throws InterruptedException
    {

        robot.init(hardwareMap);
        //loads hardwareMap

        double drive;
        double turn;
        double leftValue;
        double rightValue;
        double powerFL;
        double powerFR;
        double powerRL;
        double powerRR;
        double armPower;
        boolean runIntake = false;
        boolean reverseIntake = false;
        boolean slowIntake = false;
        double speed = 0.5;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Shock drone going live!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "ASSUMING DIRECT CONTROL");
        telemetry.update();

        while(opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;


            // Combine drive and turn for blended motion.
            leftValue  = -drive + turn;
            rightValue = drive - turn;
            powerFL = leftValue;
            powerFR = rightValue;
            powerRL = leftValue;
            powerRR = rightValue;

            //apply acceleration curve for additional driver control
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
            //both gamepads can control the arm
            //gamepad2 can use left stick for fine arm control
            armPower = (((gamepad1.right_trigger+gamepad2.right_trigger)+(0.1*-gamepad2.left_stick_y))-(gamepad1.left_trigger+gamepad2.left_trigger));
            armPower *= ARM_SPEED;

            //sets maxes for each value
            armPower = Range.clip(armPower, -ARM_SPEED, ARM_SPEED);

            robot.armL.setPower(armPower);
            robot.armR.setPower(armPower);

            if(gamepad1.a == true){
                runIntake = true;
                reverseIntake = false;
                slowIntake = false;
            }
            if(gamepad1.x == true) {
                runIntake = false;
                reverseIntake = true;
                slowIntake = false;
            }
            if(gamepad1.b == true) {
                runIntake = false;
                reverseIntake = false;
                slowIntake = false;
            }
            if(gamepad1.a == true){
                runIntake = true;
                reverseIntake = false;
                slowIntake = false;
            }
            if(gamepad1.y == true){
                runIntake = false;
                reverseIntake = false;
                slowIntake = true;
            }
            if(gamepad1.dpad_up == true){
                speed = 1;
            }
            if(gamepad1.dpad_down==true) {
                speed = 0.5;
            }
            if(runIntake){
                robot.intakeL.setPower(INTAKE_SPEED);
                robot.intakeR.setPower(INTAKE_SPEED);
            } else if (reverseIntake) {
                robot.intakeL.setPower(-0.1);
                robot.intakeR.setPower(-0.1);
            } else if (slowIntake){
                robot.intakeL.setPower(0.1);
                robot.intakeR.setPower(0.1);
            } else {
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }



            telemetry.addData("Status", "Left: "+ leftValue+"        Right: "+ rightValue+"\n" +
                    "Power: "+ drive +"        Turn: "+turn+"\n"+
            "Arm Power: "+armPower+"     Intake Power: "+INTAKE_SPEED);
            telemetry.update();
    }
}




                    //idle();
}