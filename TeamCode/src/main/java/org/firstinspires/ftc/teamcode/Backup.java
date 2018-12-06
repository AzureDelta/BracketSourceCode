package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Drive and Dump)", group = "Auto")

/* Declare OpMode members. */

public class Backup extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig();

    private ElapsedTime runtime = new ElapsedTime();

    public static final double DRIVE_SPEED = 0.5;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Insertion checklist complete. All systems GO.");
        telemetry.update();

        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////

        //write main portion of the opMode here

        telemetry.addData("Status", "I've got a good lock! Firing!");
        telemetry.update();

        //ONE TILE IS 24 INCHES X 24 INCHES

        //drive through
        //current implementation of rotation count is a placeholder
        drive(0.5, 1.6);
        //recenters based on the value of offset
/*
        strafe(0.5, -OFFSET * 0.25);
*/

        //drive into the depot
        //drive(0.5, M * 1 * FEET);

        intake(0.9, 2.25);
    }

    public void strafe(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.update();
            robot.motorFL.setPower(-speed);
            robot.motorFR.setPower(-speed);
            robot.motorRL.setPower(speed);
            robot.motorRR.setPower(speed);
        }
    }
    public void actuate(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "actuating     Time Left: " + time, runtime.seconds());
            telemetry.update();
            robot.actuator.setPower(speed);
        }
    }

    public void intake(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "intaking     Time Left: " + time, runtime.seconds());
            telemetry.update();
            robot.intakeR.setPower(speed);
        }
    }


    public void drive(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "driving     Time Left: " + time, runtime.seconds());
            telemetry.update();
            robot.motorFL.setPower(speed);
            robot.motorFR.setPower(speed);
            robot.motorRL.setPower(speed);
            robot.motorRR.setPower(speed);
        }
    }
}
