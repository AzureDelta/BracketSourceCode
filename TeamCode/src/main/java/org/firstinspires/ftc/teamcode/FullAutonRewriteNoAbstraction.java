package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.FullAutonomousRewriteTest.INCHES;
import static org.firstinspires.ftc.teamcode.FullAutonomousRewriteTest.M;

@Autonomous(name="FART NoAbstraction", group="Auto")

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

        //drive, strafe
        robot.drive(0.5, 12 * INCHES * M);
        sleep(500);
        robot.drive(0.5, -12 * INCHES * M);
        sleep(500);
        //positive dist. is right
        //negative dist. is left
        //first right 3 inches
        robot.strafe(0.5, 12 * INCHES * M);
        sleep(500);
        //then left 3 inches
        robot.strafe(0.5, -12 * INCHES * M);
        sleep(500);

    }

}