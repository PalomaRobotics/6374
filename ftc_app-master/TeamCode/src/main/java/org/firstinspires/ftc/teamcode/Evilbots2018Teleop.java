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

@TeleOp(name="Evilbots2019 TeleOp", group="Linear Opmode")
//@Disabled
public class Evilbots2018Teleop extends LinearOpMode {

    // Misc.
    private ElapsedTime runtime = new ElapsedTime();
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cRangeSensor distance = null;
    private boolean dunk = false, latch = false;
    double tempTime = 0;


    //Drive Variables
    private DcMotor midLeftDrive = null;
    private DcMotor midRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private double throttle = 1;

    //Lift Motors
    private DcMotor chainLift = null;
    private DcMotor dunkLift = null;
    private double liftSpeed = 1;
    private boolean control = true;
    boolean stage1 = true, stage2 = false;


    //Intake Variables
    private DcMotor ramp = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        midLeftDrive  = hardwareMap.get(DcMotor.class, "mid_left_drive");
        midRightDrive = hardwareMap.get(DcMotor.class, "mid_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class,"back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        chainLift = hardwareMap.get(DcMotor.class, "lift");
        dunkLift = hardwareMap.get(DcMotor.class, "dunk");
        ramp = hardwareMap.get(DcMotor.class,"ramp");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        midLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        midRightDrive.setDirection(DcMotor.Direction.REVERSE);
        chainLift.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

                //Driving Mechanics (One stick)
                /*
                if(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2)
                {
                    midRightDrive.setPower(gamepad1.left_stick_x / 1.4);
                    midLeftDrive.setPower(-gamepad1.left_stick_x / 1.4);
                }
                else
                {
                    midLeftDrive.setPower(gamepad1.left_stick_y / 1.4);
                    midRightDrive.setPower(gamepad1.left_stick_y / 1.4);
                }
                */

            //Dive Mechanics (Tank)
            if(gamepad1.right_trigger > 0.2)
            {
                midRightDrive.setPower(gamepad1.right_stick_y / 2);
                midLeftDrive.setPower(gamepad1.left_stick_y / 2);
//its free real estate
            }
            else{
                midRightDrive.setPower(gamepad1.right_stick_y / 1.4);
                midLeftDrive.setPower(gamepad1.left_stick_y / 1.4);
                backRightDrive.setPower(gamepad1.right_stick_y / 1.4);
                backLeftDrive.setPower(gamepad1.left_stick_y / 1.4);
            }

            if(gamepad1.x)
            {
                ramp.setPower(-0.25);
                midLeftDrive.setPower(0.4); //Adjusts how fast it goes in reverse when lifting up the ramp
                midRightDrive.setPower(0.4); //^^^
            }
            else if(gamepad1.y)
            {
                ramp.setPower(0.15);
                midLeftDrive.setPower(0.2); //Adjusts how fast it goes in reverse when lifting up the ramp
                midRightDrive.setPower(0.2); //^^^
            }
            else
            {
                ramp.setPower(0);
            }

                //Lift Mechanics
                if (gamepad1.dpad_up && chainLift.getCurrentPosition() < 2950) {
                    chainLift.setPower(liftSpeed);
                } else if (gamepad1.dpad_down && chainLift.getCurrentPosition() > 0) {
                    chainLift.setPower(-liftSpeed);
                } else {
                    chainLift.setPower(0);
                }

                //Dunk Mechanics
                if (gamepad1.a) {
                    dunkLift.setPower(-0.7);
                } else if (gamepad1.b) {
                    dunkLift.setPower(0.7);
                } else {
                    dunkLift.setPower(0);
                }

            // Display important variables to the remote
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Lift: ", chainLift.getCurrentPosition());
            telemetry.addData("Dunk", dunkLift.getCurrentPosition());
            telemetry.update();
        }
    }
}