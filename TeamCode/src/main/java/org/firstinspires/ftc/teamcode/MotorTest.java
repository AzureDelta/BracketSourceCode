package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "THIS ONE", group = "AAAAAAAAAAAA")

/* Declare OpMode members. */

public class MotorTest extends LinearOpMode {
    TestMap robot = new TestMap();

    private ElapsedTime runtime = new ElapsedTime();

    public static final double DRIVE_SPEED = 0.5;
    public static final double INTAKE_SPEED = 0.9;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Brandon", "is cancer");
        telemetry.update();

        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////

        //write main portion of the opMode here


        //ONE TILE IS 24 INCHES X 24 INCHES

        //drive through
        //current implementation of rotation count is a placeholder
        drive(DRIVE_SPEED, 5);
        //recenters based on the value of offset
/*
        strafe(0.5, -OFFSET * 0.25);
*/

        //drive into the depot
        //drive(0.5, M * 1 * FEET);

    }

    public void strafe(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.update();
            robot.motorFR.setPower(-speed);

        }
    }

    public void drive(double speed, double time) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Status:", "driving     Time Left: " + time, runtime.seconds());
            telemetry.update();
            robot.motorFR.setPower(speed);
        }
    }
}
