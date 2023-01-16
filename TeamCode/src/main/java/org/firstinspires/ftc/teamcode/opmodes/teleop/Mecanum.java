package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.TurtleRobotTeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//  Controls:
// left stick forward and backward
// right stick left and right to strafe
// left stick left and right to turn
// a to move linear slide up
// b to m-ove linear slide down

@TeleOp
public class Mecanum extends LinearOpMode {

    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive, armServo, clawServo;
    double driveSpeed = 0.8;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurtleRobotTeleOp robot = new TurtleRobotTeleOp(this);
        robot.init(hardwareMap);
        robot.clawServo.setPosition(0);
        waitForStart();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y ,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.7
                    )
            );
            drive.update();
//
            robot.leftslidemotor.setPower((gamepad2.right_trigger - gamepad2.left_trigger));
            robot.rightslidemotor.setPower((gamepad2.right_trigger - gamepad2.left_trigger));
            while (gamepad2.dpad_down) {robot.armServo.setPosition(0);}
            while (gamepad2.dpad_up) {robot.armServo.setPosition(1);}
            while (gamepad2.dpad_right) {robot.armServo.setPosition(0.7);}
            while (gamepad2.dpad_left) {robot.armServo.setPosition(0.5);}
            while (gamepad2.a) {robot.clawServo.setPosition(0);}
            while (gamepad2.b) {robot.clawServo.setPosition(1);}


            telemetry.addLine("motor name               motor speed");
            telemetry.addLine();
            telemetry.addData("Front right drive power = ", frontRightDrive);
            telemetry.addData("Front left drive power  = ", frontLeftDrive);
            telemetry.addData("Back right drive power  = ", backRightDrive);
            telemetry.addData("Back left drive power   = ", backLeftDrive);
            telemetry.update();
        }
    }
}

