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

package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Evilbots2018 TeleOp", group="Linear Opmode")
@Disabled
public class Evilbots2018Teleop extends LinearOpMode {

    // Misc.
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cRangeSensor distance = null;
    private boolean lastAState;

    //Drive Variables
    private DcMotor midLeftDrive = null;
    private DcMotor midRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    //Lift Motors
    private DcMotor chainLift = null;
    private DcMotor dunkLift = null;
    private double liftSpeed = 0.5;

    //Intake Variables
    private DcMotor intakeSweep = null;
    private Servo leftIntake = null;
    private Servo rightIntake = null;
    private double intakeSpeed = 0.5;
    private  boolean rightUp = true, leftUp = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        midLeftDrive  = hardwareMap.get(DcMotor.class, "mid_left_drive");
        midRightDrive = hardwareMap.get(DcMotor.class, "mid_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeSweep = hardwareMap.get(DcMotor.class, "intake_sweep");

        leftIntake = hardwareMap.get(Servo.class, "left_intake");
        rightIntake = hardwareMap.get(Servo.class, "right_intake");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        midLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        midRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);

        //Start motors/servos in beginning positions
        rightIntake.setPosition(90);
        leftIntake.setPosition(90);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Driving Mechanics
            midLeftDrive.setPower(gamepad1.left_stick_y);
            backLeftDrive.setPower(gamepad1.left_stick_y);
            midRightDrive.setPower(gamepad1.right_stick_y);
            backRightDrive.setPower(gamepad1.right_stick_y);

            //Intake Sweep Mechanics
            if (gamepad1.a && !lastAState) {
                rightIntake.setPosition(0);
                leftIntake.setPosition(0);
                double rotation = intakeSweep.getCurrentPosition();
                for (rotation = 0; rotation < (rotation + 360); rotation = rotation)
                {
                    intakeSweep.setPower(intakeSpeed);
                }
            }
            else
            {
                intakeSweep.setPower(0);
            }

            //Intake Box Mechanics
            if(gamepad1.dpad_right)
            {
                if(!rightUp)
                {
                    rightIntake.setPosition(180);
                    rightUp = true;
                }
                else if(rightUp)
                {
                    rightIntake.setPosition(90);
                    rightUp = false;
                }
            }
            if(gamepad1.dpad_left)
            {
                if(!leftUp)
                {
                    leftIntake.setPosition(180);
                    leftUp = true;
                }
                else if(leftUp)
                {
                    rightIntake.setPosition(90);
                    leftUp = false;
                }
           }

            //Lift Mechanics
            if(gamepad1.dpad_up && chainLift.getCurrentPosition() < 1000)
            {
                chainLift.setPower(-liftSpeed);
            }
            else if(gamepad1.dpad_down && chainLift.getCurrentPosition() > 0)
            {
                chainLift.setPower(liftSpeed);
            }
            else
            {
                chainLift.setPower(0);
            }

            //Swing Mechanics
            if(gamepad1.left_bumper)
            {
                encoder.RunToEncoderDegree(dunkLift, 90, 0.5);
            }
            else if(gamepad1.right_bumper)
            {
                encoder.RunToEncoderDegree(dunkLift, 0, 0.5);
            }

            //Get last state of important buttons on the controller
            lastAState = gamepad1.a;

            // Display important variables to the remote
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}