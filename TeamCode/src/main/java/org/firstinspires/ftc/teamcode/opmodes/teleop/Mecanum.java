package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
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
    private ElapsedTime runtime = new ElapsedTime();
    double frontLeftDrive, frontRightDrive, backRightDrive, backLeftDrive, armServo, clawServo;
    double driveSpeed = 0.8;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES   =  4.409;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurtleRobotTeleOp robot = new TurtleRobotTeleOp(this);
        robot.init(hardwareMap);
        robot.leftslidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightslidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.clawServo.setPosition(1);
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
            while (gamepad2.a) {robot.armServo.setPosition(0.1);}
            while (gamepad2.b) {robot.armServo.setPosition(0.73);}
//            while (gamepad2.dpad_right) {robot.armServo.setPosition(0.7);}
//            while (gamepad2.dpad_left) {robot.armServo.setPosition(0.5);}
            while (gamepad2.left_bumper) {robot.clawServo.setPosition(0.5);}
            while (gamepad2.right_bumper) {robot.clawServo.setPosition(1);} // Close

            while (gamepad2.dpad_down) {encoderLinearSlide(robot, 1.0, 0,2.1);}
            while (gamepad2.dpad_up) {encoderLinearSlide(robot, 1.0, -115, 2.1);}
            while (gamepad2.dpad_left) {encoderLinearSlide(robot, 1.0, -75, 2);}
            while (gamepad2.dpad_right) {encoderLinearSlide(robot, 1.0, -50, 2);}


            telemetry.addLine("motor name               motor speed");
            telemetry.addLine();
            telemetry.addData("Front right drive power = ", frontRightDrive);
            telemetry.addData("Front left drive power  = ", frontLeftDrive);
            telemetry.addData("Back right drive power  = ", backRightDrive);
            telemetry.addData("Back left drive power   = ", backLeftDrive);
            telemetry.update();
        }
    }
    public void encoderLinearSlide(TurtleRobotTeleOp robot, double speed, double sinch,
                                     double timeoutS) {
        int newSlideTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newSlideTarget = robot.leftslidemotor.getCurrentPosition() * (int) (sinch * COUNTS_PER_INCH);
            telemetry.addLine("Target:" + String.valueOf(newSlideTarget));
            telemetry.addLine("Current:" + String.valueOf(robot.leftslidemotor.getCurrentPosition()));
            telemetry.update();

            robot.leftslidemotor.setTargetPosition(newSlideTarget);
            robot.rightslidemotor.setTargetPosition(newSlideTarget);

            robot.leftslidemotor.setPower(Math.abs(speed));
            robot.rightslidemotor.setPower(Math.abs(speed));

            // Turn On RUN_TO_POSITION
            robot.leftslidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightslidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            sleep(50);   // optional pause after each move

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.leftslidemotor.isBusy() || (robot.rightslidemotor.isBusy()))) {
                telemetry.addLine("Target:" + String.valueOf(newSlideTarget));
                telemetry.addLine("Current:" + String.valueOf(robot.leftslidemotor.getCurrentPosition()));
                telemetry.update();
            }

            // Stop all motion;
            robot.leftslidemotor.setPower(0);
            robot.rightslidemotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
        telemetry.addLine("Final:" + String.valueOf(robot.leftslidemotor.getCurrentPosition()));
        telemetry.update();
    }
}

