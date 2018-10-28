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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class AutonomousControls {

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION;     //used to compute degrees

    HardwareConfig robot = new HardwareConfig();

    public void rotateIntake(double speed, double distance, double timeout){
        int targetL;
        int targetR;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        targetL = robot.armL.getCurrentPosition() + (int) (distance);
        targetR = robot.armR.getCurrentPosition() + (int) (distance);
        robot.armL.setTargetPosition(targetL);
        robot.armR.setTargetPosition(targetR);

        // Turn On RUN_TO_POSITION
        robot.armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.armL.setPower(Math.abs(speed));
        robot.armR.setPower(Math.abs(speed));;

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

        // Stop all motion;
        robot.armL.setPower(0);
        robot.armR.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateArm(double speed, double distance){
        int targetL;
        int targetR;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        targetL = robot.armL.getCurrentPosition() + (int) (distance);
        targetR = robot.armR.getCurrentPosition() + (int) (distance);
        robot.armL.setTargetPosition(targetL);
        robot.armR.setTargetPosition(targetR);

        // Turn On RUN_TO_POSITION
        robot.armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.armL.setPower(Math.abs(speed));
        robot.armR.setPower(Math.abs(speed));;

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
            robot.armL.setPower(0);
            robot.armR.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.armL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void drive(double speed, double distance) {
        //declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

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

    public void turn(double speed, double angle){
        //declares target point storage variables
        int targetFL;
        int targetFR;
        int targetRL;
        int targetRR;

        //rotates counter clockwise based on angle
        targetFL = robot.motorFL.getCurrentPosition() + (int) (COUNTS_PER_ROTATION-angle);
        targetFR = robot.motorFR.getCurrentPosition() + (int) (COUNTS_PER_ROTATION+angle);
        targetRL = robot.motorRL.getCurrentPosition() + (int) (COUNTS_PER_ROTATION-angle);
        targetRR = robot.motorRR.getCurrentPosition() + (int) (COUNTS_PER_ROTATION+angle);
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
