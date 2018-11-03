package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

//*In theory* this should also be compatible with tank drive.

@TeleOp(name ="WestCoastDrive", group ="TeleOp")
public class WestCoastDrive extends LinearOpMode{
    public static final double SPEED = 0.5;

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
        double armPower;
        int intakeToggle;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Shock drone going live!");
        telemetry.update();

        waitForStart();

        telemetry.addData("Say", "ASSUMING DIRECT CONTROL");
        telemetry.update();

        while(opModeIsActive())
        {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            leftValue  = -(drive + turn);
            rightValue = -(drive - turn);

            armPower = (gamepad1.right_trigger-gamepad1.left_trigger);

            if(gamepad1.a){
                robot.intakeL.setPower(0.1);
                robot.intakeR.setPower(0.1);
            }
            else if (gamepad1.b) {
                robot.intakeL.setPower(-0.1);
                robot.intakeR.setPower(-0.1);
            }

            //sets maxes for each value
            leftValue = Range.clip(leftValue, -SPEED, SPEED);
            rightValue = Range.clip(rightValue, -SPEED, SPEED);
            armPower = Range.clip(armPower, -0.1, 0.1);



            robot.motorFL.setPower(leftValue);
            robot.motorFR.setPower(rightValue);
            robot.motorRL.setPower(leftValue);
            robot.motorRR.setPower(rightValue);
            robot.armL.setPower(armPower);
            robot.armR.setPower(armPower);

            telemetry.addData("Say", "Left: "+ leftValue+"        Right: "+ rightValue+"\n" +
                    "Power: "+ drive +"        Turn: "+turn);
            telemetry.update();

            //idle();
        }
    }
}

