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

    public static void drive(double speed, double time){

    }

    public static void turn(double speed, double angle){

    }
}
