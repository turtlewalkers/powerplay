package org.firstinspires.ftc.teamcode.opmodes.tests;

import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="basic controls")
@Disabled
public class basic_controls extends LinearOpMode{
    TurtleRobotAuto robot = new TurtleRobotAuto(this);
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            straight( 0.5, 5000);
            stopRobot();
//            sleep(1000);
//            strafeLeft(0.25, 1000);
//            stopRobot();sleep(1000);
//            strafeRight(0.25, 1000);
//            stopRobot();
//            left(0.25, 1000);
//            stopRobot();
//            right(0.75, 500);
//            stopRobot();
        }
    }
//    public void ServoArm(double power, int time) {
//        robot.ArmServo.setPower(power);
//        sleep(time);
//    }
    public void LinearSlide(double power, int time) {
        // Negative power = up
//        robot.leftslidemotor.setPower(power);
        robot.rightslidemotor.setPower(power);
        sleep(time);
    }
    public void straight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot.rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void strafeRight(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(-power);
        robot.rightfrontmotor.setPower(-power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void strafeLeft(double power, int time) {
        robot.leftfrontmotor.setPower(-power);
        robot.leftbackmotor.setPower(power);
        robot. rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(-power);
        sleep(time);
    }
    public void left(double power, int time) {
        robot.leftfrontmotor.setPower(power);
        robot.leftbackmotor.setPower(power);
        robot. rightfrontmotor.setPower(-power);
        robot.rightbackmotor.setPower(-power);
        sleep(time);
    }
    public void right(double power, int time) {
        robot.leftfrontmotor.setPower(-power);
        robot.leftbackmotor.setPower(-power);
        robot. rightfrontmotor.setPower(power);
        robot.rightbackmotor.setPower(power);
        sleep(time);
    }
    public void stopRobot() {
        robot.leftfrontmotor.setPower(0);
        robot.leftbackmotor.setPower(0);
        robot.rightfrontmotor.setPower(0);
        robot.rightbackmotor.setPower(0);
    }
}