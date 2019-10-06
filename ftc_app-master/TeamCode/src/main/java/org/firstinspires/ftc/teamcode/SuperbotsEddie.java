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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Superbots18 Eddie's Mayhem", group="Competition")
//@Disabled
//Created by Brendan Conwell
public class SuperbotsEddie extends LinearOpMode {

    //region Variable Declaration
    // Misc. Variables
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    private boolean stage1 = true, stage2 = false, stage3 = false, stage4 = false, stage5 = false;

    //Drive Variables
    DcMotor leftDrive, rightDrive;

    //Conveyor Variables
    DcMotor conveyorPivot, conveyorLeft, conveyorRight;
    double pivotSpeed = 0.5, conveyorSpeed =0.2;

    //Extend Variables
    DcMotor extendLeft, extendRight;
    double extendSpeed = 0.5;

    //Claw Variables
    Servo claw;

    //Sensor Variables
    private ModernRoboticsI2cColorSensor colorSensor = null;
    private ModernRoboticsI2cIrSeekerSensorV3 irSensor = null;
    private ModernRoboticsI2cRangeSensor rangeSensor = null;
    private ModernRoboticsI2cGyro gyroSensor = null;
    private ModernRoboticsAnalogOpticalDistanceSensor odsSensor = null;

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
        //conveyorLeft = hardwareMap.dcMotor.get("conveyor_left");
        //conveyorRight = hardwareMap.dcMotor.get("conveyor_right");
        //conveyorPivot = hardwareMap.dcMotor.get("conveyor_pivot");
        //claw = hardwareMap.servo.get("claw");
        irSensor = hardwareMap.get(ModernRoboticsI2cIrSeekerSensorV3.class, "ir_sensor");
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        gyroSensor = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro_sensor");
        odsSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "ods_sensor");

        int controlerScheme = 1;

        GyroControlClass gyroControl = new GyroControlClass(gyroSensor);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        //conveyorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //conveyorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //conveyorPivot.setDirection(DcMotorSimple.Direction.FORWARD);

        boolean bLastPressed = false;
        float range = .2f;
        int iRange = 2;
        //endregion

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //inintial value of sensor
            double intSensor = odsSensor.getRawLightDetected();


            //region Jewel (Stages 1-???)
            while(stage1 && opModeIsActive()) //Turn the robot to face jewels
            {
                //telemetry.addData("Stage 1: ",stage1);
                //telemetry.addData("Gyro: ", gyroSensor.getHeading());
                telemetry.addData("ODS Raw: ", odsSensor.getRawLightDetected());
                telemetry.update();



                if (gamepad1.a)
                    intSensor = odsSensor.getRawLightDetected();

                if (gamepad1.b && bLastPressed == false)
                {
                    if (iRange == 2)
                    {
                        iRange = 3;
                        range = .3f;
                    }
                    else if (iRange == 3)
                    {
                        iRange = 1;
                        range = .1f;
                    }
                    else if (iRange == 1)
                    {
                        iRange = 2;
                        range = .2f;
                    }
                }
                //telemetry.addData("Sensor Range", range);
                telemetry.addData( "Drive function", controlerScheme);

                if (gamepad1.b == false)
                    bLastPressed = false;
                else
                    bLastPressed = true;

                if (gamepad1.dpad_up)
                    controlerScheme = 1;
                if (gamepad1.dpad_down)
                    controlerScheme = 2;
                if (gamepad1.dpad_left)
                    controlerScheme = 3;
                if (gamepad1.dpad_right)
                    controlerScheme = 4;

            if (controlerScheme == 1) {
                telemetry.addData("Use Right Stick to Drive", null);

                if (gamepad1.right_stick_y > .5 || gamepad1.right_stick_y < -.5) {
                    rightDrive.setPower(-gamepad1.right_stick_y);
                    leftDrive.setPower(-gamepad1.right_stick_y);
                } else if (gamepad1.left_stick_x > .5) {
                    rightDrive.setPower(gamepad1.right_stick_x);
                    leftDrive.setPower(-gamepad1.right_stick_x);
                } else if (gamepad1.left_stick_x > -.5) {
                    rightDrive.setPower(-gamepad1.right_stick_x);
                    leftDrive.setPower(gamepad1.right_stick_x);
                }
                else
                {
                    rightDrive.setPower(0);
                    leftDrive.setPower(0);
                }
            }
                if (controlerScheme == 2) {
                telemetry.addData("Use Shoulder Buttons to Drive", null);
                    if (gamepad1.left_trigger > .1) {
                        leftDrive.setPower(gamepad1.left_trigger);
                        telemetry.addData("left trigger", gamepad1.left_trigger);
                    } else if (gamepad1.left_bumper)
                        leftDrive.setPower(-.5);
                    else
                        leftDrive.setPower(0);
                    if (gamepad1.right_trigger > .1) {
                        rightDrive.setPower(gamepad1.right_trigger);
                        telemetry.addData("right trigger", gamepad1.right_trigger);
                    } else if (gamepad1.right_bumper)
                        rightDrive.setPower(-.5);
                    else
                        rightDrive.setPower(0);
                }

                telemetry.addData("Press 'a' toSset to Line Material", null);
                telemetry.addData("Line Sensor Range:", range);
                if (controlerScheme == 3)
                {
                    gyroControl.FollowXLine(intSensor, leftDrive, rightDrive, -.5, odsSensor.getRawLightDetected(), range);
                }
                if (controlerScheme == 4) {

                    if(irSensor.getAngle() > 30)
                    {
                        leftDrive.setPower(0.55);
                        rightDrive.setPower(0.1);
                    }
                    else if(irSensor.getAngle() > 75)
                    {
                        leftDrive.setPower(0.85);
                        rightDrive.setPower(0.1);
                    }
                    else if(irSensor.getAngle() < -30)
                    {
                        leftDrive.setPower(0.1);
                        rightDrive.setPower(0.55);
                    }
                    else if (irSensor.getAngle() < -75)
                    {
                        leftDrive.setPower(0.1);
                        rightDrive.setPower(0.85);
                    }
                    else if(irSensor.getAngle() < 25 && irSensor.getAngle()> -25 && irSensor.signalDetected())
                    {
                        leftDrive.setPower(0.15);
                        rightDrive.setPower(0.15);
                    }
                    else
                    {
                        leftDrive.setPower(0);
                        rightDrive.setPower(0);
                    }

                    //region Telemetry Data
                    telemetry.update();
                    telemetry.addData("IR Sensor: ", irSensor.getAngle());
                    telemetry.addData("IR Detected: ", irSensor.signalDetected());
                    //endregion
                }

                /*
                gyroControl.Rotate(leftDrive,rightDrive,0.2,270);
                if(gyroSensor.getHeading() >265 && gyroSensor.getHeading()<275)
                {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    stage2 = true;
                    stage1 = false;
                }
                */
            }
            while(stage2 && opModeIsActive())
            {
                telemetry.addData("Stage 2: ",stage2);
                telemetry.addData("Range: ",rangeSensor.cmUltrasonic());
                telemetry.update();

                if(rangeSensor.cmUltrasonic() > 125)
                {
                    rightDrive.setPower(0.2);
                    leftDrive.setPower(0.2);
                }
                else
                {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    stage3 = true;
                    stage2 = false;
                }
            }
            while(stage3 && opModeIsActive())
            {
                telemetry.addData("Stage 3: ",stage3);
                telemetry.addData("Color Sensor Red: ", colorSensor.red());
                telemetry.addData("Color Sensor Blue: ", colorSensor.blue());
                telemetry.addData("ODS Raw: ", odsSensor.getRawLightDetected());
                telemetry.addData("ODS: ", odsSensor.getLightDetected());
                telemetry.update();

                if(colorSensor.red() > 3 && colorSensor.blue() < 3)
                {
                    telemetry.addData("The object is: ", "Red");
                }
                else if(colorSensor.blue() > 3 && colorSensor.red() < 3)
                {
                    telemetry.addData("The object is: ", "Blue");
                }
            }
            //endregion
            //region Telemetry Data
            telemetry.update();
            //endregion
        }
    }

}

