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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LandAndSort {

    AutonomousControls auto = new AutonomousControls(); //AutonomousControls

    private GoldAlignDetector detector;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder (40:1)
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_ROTATION = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;     //used to compute degrees
    static final double INCHES = (COUNTS_PER_MOTOR_REV * 0.5) / (WHEEL_DIAMETER_INCHES * Math.PI); //calculates counts per inch
    static final double FEET = 12 * INCHES; //calculates counts per foot
    static final double DEGREES = (1120) / 360; //calculates counts per degree

    public void partOne() {

        //write main portion of the opMode here

        //lower the robot
        auto.rotateArm(0.5, 1120);

        //declare counter variable
        int rotationCount = 0;

        //runs loop until robot is aligned with mineral
        while (detector.getAligned() != true) {
            if (detector.getXPosition() < 170) {
                auto.turn(0.25, -3 * DEGREES);
                rotationCount--;

            } else if (detector.getXPosition() > 170) {
                auto.turn(0.25, 3 * DEGREES);
                rotationCount++;

            } else {
                //brandon i'm useless but would this work
                return;
                //nope, we don't want it to accidentally ram particles i think
                //figure out how to abort the program here
            }

            //drive to crater
            //current implementation of rotation count is a placeholder
            auto.drive(0.5, 19 * INCHES);
            auto.turn(0.25, 2*3 * rotationCount * DEGREES);
        }
    }
}