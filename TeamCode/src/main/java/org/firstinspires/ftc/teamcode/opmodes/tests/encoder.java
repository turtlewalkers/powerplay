/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.TurtleRobotAuto;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Encoder", group="Tests")
@Disabled
public class encoder extends LinearOpMode {

    /* Declare OpMode members. */
    TurtleRobotAuto robot = new TurtleRobotAuto(this);
    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   =  3.7795276;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(robot, DRIVE_SPEED,  12,  12, 12, 12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(robot, TURN_SPEED,   12, -12, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(robot, DRIVE_SPEED, -24, 24, 24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(TurtleRobotAuto turtleRobotAuto, double speed,
                             double leftfrontInches, double leftbackInches,
                             double rightfrontInches, double rightbackInches,
                             double timeoutS) {
        int newLeftfrontTarget;
        int newLeftbackTarget;
        int newRightfrontTarget;
        int newRightbackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfrontTarget = robot.leftfrontmotor.getCurrentPosition() + (int) (leftfrontInches * COUNTS_PER_INCH);
            newLeftbackTarget = robot.leftbackmotor.getCurrentPosition() + (int) (leftbackInches * COUNTS_PER_INCH);
            newRightfrontTarget = robot.rightfrontmotor.getCurrentPosition() + (int) (rightfrontInches * COUNTS_PER_INCH);
            newRightbackTarget = robot.rightbackmotor.getCurrentPosition() + (int) (rightbackInches * COUNTS_PER_INCH);
            robot.leftfrontmotor.setTargetPosition(newLeftfrontTarget);
            robot.leftbackmotor.setTargetPosition(newLeftfrontTarget);
            robot.rightfrontmotor.setTargetPosition(newRightfrontTarget);
            robot.rightbackmotor.setTargetPosition(newRightbackTarget);


            // Turn On RUN_TO_POSITION
            robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftfrontmotor.setPower(Math.abs(speed));
            robot.leftbackmotor.setPower(Math.abs(speed));
            robot.rightfrontmotor.setPower(Math.abs(speed));
            robot.rightbackmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    && (robot.leftfrontmotor.isBusy() &&
                    robot.leftbackmotor.isBusy()
                    && robot.rightfrontmotor.isBusy()
                    && robot.rightbackmotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d",
                        newLeftfrontTarget,
                        newLeftbackTarget,
                        newRightfrontTarget,
                        newRightbackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftfrontmotor.getCurrentPosition(),
                        robot.leftbackmotor.getCurrentPosition(),
                        robot.rightfrontmotor.getCurrentPosition(),
                        robot.rightbackmotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftfrontmotor.setPower(0);
            robot.leftbackmotor.setPower(0);
            robot.rightfrontmotor.setPower(0);
            robot.rightbackmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}