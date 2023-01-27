package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous(group="drive")
public class AutoRightOnePlusThree extends LinearOpMode {
    OpenCvCamera camera;
    private ElapsedTime runtime = new ElapsedTime();
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    TurtleRobotAuto robot = new TurtleRobotAuto(this);
    static final double FEET_PER_METER = 3.28084;
    int SLIDE = 115;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     PULLEY_DIAMETER_INCHES   =  4.409;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (PULLEY_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);

        robot.leftslidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightslidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightslidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.clawServo.setPosition(1);
//        telemetry.addLine(String.valueOf(robot.leftslidemotor.getCurrentPosition()));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .forward(45)
//                .build();
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeRight(12.5)
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .forward(3)
//                .build();

        TrajectorySequence alignToHigh = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .strafeLeft(5)
                .back(44)
                .lineToLinearHeading(new Pose2d(-50, -3, Math.toRadians(65)))
                .build();

        Trajectory goToConeStack = drive.trajectoryBuilder(alignToHigh.end())
                .lineToLinearHeading(new Pose2d(-47, -12, Math.toRadians(270)))
                .build();
        Trajectory goBack = drive.trajectoryBuilder(goToConeStack.end())
                .lineToLinearHeading(new Pose2d(-50,-3,Math.toRadians(245)))
                .build();

        TrajectorySequence parkAtOne = drive.trajectorySequenceBuilder(goToConeStack.end())
                .strafeLeft(12)
                .back(4)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence parkAtTwo = drive.trajectorySequenceBuilder(goToConeStack.end())
                .strafeRight(10)
                .back(4)
                .turn(Math.toRadians(-90))
                .build();
        TrajectorySequence parkAtThree = drive.trajectorySequenceBuilder(goToConeStack.end())
                .strafeRight(32.1)
                .back(22)
                .build();


        waitForStart();

        if (isStopRequested()) return;

//        // Go to high junction
        encoderLinearSlideUp(robot, 1.0, 3);
        sleep(500);
        robot.clawServo.setPosition(0.5);
        sleep(100);
        robot.armServo.setPosition(0.73);
        encoderLinearSlideDown(robot, 1.0, 3);
        robot.clawServo.setPosition(0.5);
        drive.followTrajectory(goToConeStack);
        sleep(200);
        robot.clawServo.setPosition(0.5);
        sleep(500);
        robot.armServo.setPosition(0);

        // Cone 2, 3, 4, 5
        for (int i = 1; i <= 4; i++) {
            drive.followTrajectory(goBack);
            encoderLinearSlideUp(robot, 1.0, 3);
            sleep(100);
            robot.clawServo.setPosition(0.5);
            sleep(500);
            robot.armServo.setPosition(0.73);
            sleep(200);
            encoderLinearSlideDown(robot, 1.0, 3);
            robot.armServo.setPosition(0.76+(i*0.03));
            goToConeStack = drive.trajectoryBuilder(alignToHigh.end())
                    .lineToLinearHeading(new Pose2d(-47, 12-(i*1/2), Math.toRadians(90)))
                    .build();
            drive.followTrajectory(goToConeStack);
            sleep(200);
            robot.clawServo.setPosition(1);
            sleep(500);
            robot.armServo.setPosition(0);
        }
//        drive.followTrajectory(prepareToDrop);
//        sleep(300);
//        robot.armServo.setPosition(1);
//        sleep(300);
//        //drive.followTrajectory(traj3);
//        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//            drive.followTrajectorySequence(parkAtOne);
//        } else if (tagOfInterest.id == MIDDLE) {
//            drive.followTrajectorySequence(parkAtTwo);
//        } else {
//            drive.followTrajectorySequence(parkAtThree);
//        }
//        sleep(100);
//        LinearSlide(0.7, 1800);
//        LinearSlide(0, 0);

//        drive.followTrajectory(traj4);
//          LinearSlide(-0.75, 1700);
//       LinearSlide(0, 0);
//        LinearSlide(-0.75, 1700);
//        LinearSlide(0, 0);
//        while (slide != 1100) {
//            drive.followTrajectory(traj4);
//            drive.followTrajectory(traj1);
//            robot.ArmServo.setPower(1);
//            sleep(100);
//            robot.ArmServo.setPower(0);
//            drive.followTrajectory(traj5);
//            LinearSlide(0.75, slide);
//            robot.ArmServo.setPower(-1);
//            sleep(100);
//            robot.ArmServo.setPower(0);
//            drive.followTrajectory(traj3);
//            LinearSlide(-0.75, slide);
//            LinearSlide(0, 0);
//            slide -= 300;
//        }
//
//


    }
    public void LinearSlide(double speed, long time){
        robot.leftslidemotor.setPower(speed);
        robot.rightslidemotor.setPower(speed);
        sleep(time);

    }
    public void encoderLinearSlideUp(TurtleRobotAuto turtleRobotAuto, double speed,
                                     double timeoutS) {
        int newSlideTarget;
//        double sinch = -115;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newSlideTarget = (int)(-117 * COUNTS_PER_INCH);

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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftslidemotor.isBusy() ||
                            (robot.rightslidemotor.isBusy()))) {
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
    public void encoderLinearSlideDown(TurtleRobotAuto turtleRobotAuto, double speed,
                                       double timeoutS) {
        int newSlideTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newSlideTarget = 10;
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

            sleep(50);   // optional pause after each move
            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftslidemotor.isBusy() ||
                            (robot.rightslidemotor.isBusy()))) {
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
    public void FrontBack(double speed, long time){
        robot.leftfrontmotor.setPower(speed);
        robot.rightfrontmotor.setPower(speed);
        robot.leftbackmotor.setPower(speed);
        robot.rightbackmotor.setPower(speed);
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}