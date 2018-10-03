package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Crater", group="Auto")

/* Declare OpMode members. */


public class Crater extends LinearOpMode {

    HardwareConfig robot = new HardwareConfig();   //Configs hardware

    private ElapsedTime runtime = new ElapsedTime();

    static final double PI = 3.1415; //pi
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double DRIVE_SPEED = 0.75;     //factors straight line speed
    static final double TURN_SPEED = 0.5;     //factors turn speed
    static final double COUNTS_PER_ROTATION = 1120;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV*0.5)/(WHEEL_DIAMETER_INCHES*PI); //calculates counts per inch
    static final double FEET = 12 * INCHES; //calculates counts per foot
    static final double DEGREES = (1120)/360; //calculates counts per degree

    @Override
    public void runOpMode() {

    }

    public void drive(double speed, double distance, double timeout) {
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            targetFL = robot.motorFL.getCurrentPosition() + (int) (distance);
            targetFR = robot.motorFR.getCurrentPosition() + (int) (distance);
            targetRL = robot.motorRL.getCurrentPosition() + (int) (distance);
            targetRR = robot.motorRR.getCurrentPosition() + (int) (distance);
            robot.motorFL.setTargetPosition(targetFL);
            robot.motorFR.setTargetPosition(targetFR);
            robot.motorRL.setTargetPosition(targetRL);
            robot.motorRR.setTargetPosition(targetRR);

            // Turn On RUN_TO_POSITION
            robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFL.setPower(Math.abs(speed));
            robot.motorFR.setPower(Math.abs(speed));
            robot.motorRL.setPower(Math.abs(speed));
            robot.motorRR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&

                    (runtime.seconds() < timeout) &&
                    (robot.motorFL.isBusy() && robot.motorFR.isBusy()) && (robot.motorRL.isBusy() && robot.motorRR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", targetRL, targetRR, targetFL, targetFR);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFL.getCurrentPosition(),
                        robot.motorFR.getCurrentPosition(),
                        robot.motorRL.getCurrentPosition(),
                        robot.motorRR.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.motorFL.setPower(0);
            robot.motorFR.setPower(0);
            robot.motorRL.setPower(0);
            robot.motorRR.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turn(double speed, double angle){

    }
}
