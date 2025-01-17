package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurtleRobotAuto {
    /* Public OpMode members. */
    public DcMotor rightfrontmotor;
    public DcMotor rightbackmotor = null;
    public DcMotor leftfrontmotor = null;
    public DcMotor leftbackmotor = null;
    public Servo armServo = null;
    public Servo clawServo = null;
    public DcMotor leftslidemotor = null;
    public DcMotor rightslidemotor = null;
    public DistanceSensor distance = null;
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode myOpMode = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TurtleRobotAuto (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftfrontmotor = hwMap.get(DcMotor.class, "leftfrontmotor");
        leftbackmotor = hwMap.get(DcMotor.class, "leftbackmotor");
        rightfrontmotor = hwMap.get(DcMotor.class, "rightfrontmotor");
        rightbackmotor = hwMap.get(DcMotor.class, "rightbackmotor");
        armServo = hwMap.get(Servo.class, "ArmServo");
        clawServo = hwMap.get(Servo.class, "ClawServo");
        leftslidemotor = hwMap.get(DcMotor.class, "leftslidemotor");
        rightslidemotor = hwMap.get(DcMotor.class, "rightslidemotor");
        distance = hwMap.get(DistanceSensor.class, "Distance");

        leftfrontmotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftbackmotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightfrontmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightbackmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftslidemotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightslidemotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armServo.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
//        armServo.setPosition(0);
        clawServo.setPosition(0);
        leftslidemotor.setPower(0);
        rightslidemotor.setPower(0);

        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rightslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}