package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Superbots18 District", group="Competition")
//@Disabled
//Created by Brendan Conwell
public class SuperbotsDistrict18TeleOp extends LinearOpMode {

    //region Variable Declaration
    // Misc. Variables
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    boolean lastAState, lastXState, lastBState, lastYState, lastRightState, lastLeftState;

    //Drive Variables
    DcMotor leftDrive, rightDrive;
    double turnSpeed = 0.65;

    //Conveyor Variables
    DcMotorSimple conveyorPivot, conveyorLeft, conveyorRight;
    double pivotSpeed = 0.7, conveyorSpeed =0.4;
    boolean conveyorOn;

    //Extend Variables
    DcMotor extendLeft, extendRight;
    boolean extendBoth;
    double extendSpeed = 0.2, extendLimit = 4000;

    //Pinball Arms
    Servo pinballLeft, pinballRight;
    boolean leftIn, rightIn;

    //endregion

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "0.1");
        telemetry.update();

        //region object instantiation
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        conveyorLeft = hardwareMap.dcMotor.get("conveyor_left");
        conveyorRight = hardwareMap.dcMotor.get("conveyor_right");
        conveyorPivot = hardwareMap.dcMotor.get("conveyor_pivot");
        extendRight = hardwareMap.dcMotor.get("extend_right");
        extendLeft = hardwareMap.dcMotor.get("extend_left");
        pinballLeft = hardwareMap.servo.get("pinball_left");
        pinballRight = hardwareMap.servo.get("pinball_right");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorPivot.setDirection(DcMotorSimple.Direction.FORWARD);
        extendRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //region Drive Movement
            //Forward Movement
            if (gamepad1.right_trigger > 0) {//if up/down exceeds threshold

                leftDrive.setPower(gamepad1.right_trigger); //move based on controllers y axis values
                rightDrive.setPower(gamepad1.right_trigger);
            }
            //Backward Movement
            else if (gamepad1.left_trigger > 0) {//if up/down exceeds threshold

                leftDrive.setPower(-gamepad1.left_trigger); //move based on controllers y axis values
                rightDrive.setPower(-gamepad1.left_trigger);
            }
            //Rotate Right Movement
            else if (gamepad1.right_bumper) {//if up/down exceeds threshold

                leftDrive.setPower(turnSpeed); //move based on controllers y axis values
                rightDrive.setPower(-turnSpeed);
            }
            //Rotate Left Movement
            else if (gamepad1.left_bumper) {//if up/down exceeds threshold

                leftDrive.setPower(-turnSpeed); //move based on controllers y axis values
                rightDrive.setPower(turnSpeed);
            }
            //Stop the motors when nothing is pressed
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            //endregion

            //region Conveyor Controls
            //If the dpad is pressed up
            if (gamepad1.dpad_up) {
                conveyorPivot.setPower(pivotSpeed);
            }
            //If the dpad is pressed down
            else if (gamepad1.dpad_down) {
                conveyorPivot.setPower(-pivotSpeed);
            }
            //Else do not power the motor
            else {
                conveyorPivot.setPower(0);
            }

            //If the X button is pressed... (Pulls in)
            if (gamepad1.a && gamepad1.a != lastAState)
            {
                //Toggle the conveyor on/off
                conveyorOn = !conveyorOn;

                //If the conveyor is on...
                if(conveyorOn) {
                    //Power the conveyor to pull in
                    conveyorLeft.setPower(conveyorSpeed);
                    conveyorRight.setPower(conveyorSpeed);
                }
                //Else if the conveyor is off...
                else if(!conveyorOn)
                {
                    //Turn off the conveyor
                    conveyorLeft.setPower(0);
                    conveyorRight.setPower(0);
                }
            }
            //If the B button is pressed... (Pushes out)
            if (gamepad1.b && gamepad1.b != lastBState)
            {
                //Toggle the conveyor on/off
                conveyorOn = !conveyorOn;

                //If the conveyor is on...
                if(conveyorOn) {
                    //Power the conveyor to pull in
                    conveyorLeft.setPower(-conveyorSpeed);
                    conveyorRight.setPower(-conveyorSpeed);
                }
                //Else if the conveyor is off...
                else if(!conveyorOn)
                {
                    //Turn off the conveyor
                    conveyorLeft.setPower(0);
                    conveyorRight.setPower(0);
                }
            }

            //Get the last state of the buttons used
            lastBState = gamepad1.b;
            lastAState = gamepad1.a;
            //endregion

            //region Arm Extension Controls

            //The left joystick controls the left arm extension
            if(gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0) //if the joystick moves vertically...
            {
                if(gamepad1.left_stick_y < 0 && extendLeft.getCurrentPosition() < extendLimit) //if the joystick moves up and is not past the extend limit...
                {
                    if(extendLeft.getCurrentPosition() < extendLimit)
                    {
                        extendLeft.setPower(extendSpeed); //power the left extension motor

                    }
                    if(extendRight.getCurrentPosition() < extendLimit) {
                        extendRight.setPower(extendSpeed);
                    }
                }
                else if(gamepad1.left_stick_y > 0 && extendLeft.getCurrentPosition() > 10) //if the joystick is moves down and the arm isn't in resting position...
                {
                    if(extendLeft.getCurrentPosition() > 15) {
                        extendLeft.setPower(-extendSpeed); //power the left extension motor backwards
                    }
                    if(extendRight.getCurrentPosition() > 15) {
                        extendRight.setPower(-extendSpeed);
                    }
                }
            }
            else
            {
                extendLeft.setPower(0); //de-power the motor when the joystick is not in use
                extendRight.setPower(0);
            }

            //Get used buttons last state
            lastYState = gamepad1.y;
            //endregion

            //region Pinball Arms
            if(gamepad1.dpad_right && gamepad1.dpad_right != lastRightState)
            {
                rightIn = !rightIn;
                if(rightIn)
                {
                    pinballRight.setPosition(0.22);

                }
                else if(!rightIn)
                {
                    pinballRight.setPosition(1);
                }


            }
            if(gamepad1.dpad_left && gamepad1.dpad_left != lastLeftState)
            {
                leftIn = !leftIn;
                if(leftIn)
                {
                    pinballLeft.setPosition(0.78);
                }
                else if(!leftIn) {
                    pinballLeft.setPosition(0);
                }
            }

            lastRightState = gamepad1.dpad_right;
            lastLeftState = gamepad1.dpad_left;
            //endregion

            //region Telemetry Data
            //Displays the controls on the phone
            telemetry.addData("Driving (Left Trig forward/ Right Trig backward)", "");
            telemetry.addData("Turning (Left Bump left/ Right Bump right)" ,"");
            telemetry.addData("Conveyor (A in / B out) - Running: ", conveyorOn);
            telemetry.addData("Arm Extension (Left & Right Joysticks)" ,"");
            telemetry.update();
            //endregion
        }
    }

}

