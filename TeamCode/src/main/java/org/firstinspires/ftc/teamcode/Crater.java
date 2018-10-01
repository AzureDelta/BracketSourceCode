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

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double DRIVE_SPEED = 0.75;     //factors speed
    static final double TURN_SPEED = 0.5;     //turning fine tuning, current value is a place holder
    static final double INCHES = (COUNTS_PER_MOTOR_REV*0.5)/(WHEEL_DIAMETER_INCHES*3.1415); //calculates counts per inch
    static final double FEET = 12 * INCHES; //calculates counts per foot

    @Override
    public void runOpMode() {

    }
}
