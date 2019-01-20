package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="FART NoAbstraction", group="F.A.R.T.")

/* Declare OpMode members. */


public class FullAutonRewriteNoAbstraction extends LinearOpMode {

    AutonMap robot = new AutonMap();

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        //send telemetry
        telemetry.addData("Status", "Ready to run Test Autonomous");
        telemetry.update();

        waitForStart();

        // Resets encoder values so that it doesn't attempt to run to outdated values
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;
        // Determines new target position, and pass to motor controller
        targetFL = robot.motorFL.getCurrentPosition() + 1000;
        targetFR = robot.motorFR.getCurrentPosition() + 1000;
        targetRL = robot.motorRL.getCurrentPosition() + 1000;
        targetRR = robot.motorRR.getCurrentPosition() + 1000;
        robot.motorFL.setTargetPosition(targetFL);
        robot.motorFR.setTargetPosition(targetFR);
        robot.motorRL.setTargetPosition(targetRL);
        robot.motorRR.setTargetPosition(targetRR);

        // Sets motors to run to a given encoder value
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motors are set to run at a certain speed until one reaches its target position
        while (robot.motorFL.isBusy() && robot.motorFR.isBusy() && robot.motorRL.isBusy() && robot.motorRR.isBusy()) {
            robot.motorFL.setPower(Math.abs(1));
            robot.motorFR.setPower(Math.abs(1));
            robot.motorRL.setPower(Math.abs(1));
            robot.motorRR.setPower(Math.abs(1));
            telemetry.addData("Motor FL", robot.motorFL.getCurrentPosition() + "/" + targetFL);
            telemetry.addData("Motor FR", robot.motorFL.getCurrentPosition() + "/" + targetFR);
            telemetry.addData("Motor RL", robot.motorFL.getCurrentPosition() + "/" + targetRL);
            telemetry.addData("Motor RR", robot.motorFL.getCurrentPosition() + "/" + targetRR);
            telemetry.update();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
        }
        // The motors are shutdown when a motor gets to its target position
        robot.motorFL.setPower(0);
        robot.motorFR.setPower(0);
        robot.motorRL.setPower(0);
        robot.motorRR.setPower(0);
        targetFL=0;
        targetFR=0;
        targetRL=0;
        targetRR=0;


        sleep(500);
        robot.drive(0.5, -12);
        sleep(500);
        //positive dist. is right
        //negative dist. is left
        //first right 3 inches
        robot.strafe(0.5, 12);
        sleep(500);
        //then left 3 inches
        robot.strafe(0.5, -12);
        sleep(500);

    }

}