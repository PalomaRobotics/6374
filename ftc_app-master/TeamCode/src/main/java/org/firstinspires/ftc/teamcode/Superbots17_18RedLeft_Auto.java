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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Red_Left_17/18", group="Competition")
//@Disabled
//Created by Brendan Conwell
public class Superbots17_18RedLeft_Auto extends LinearOpMode {

    // Declare all variables used in OpMode
    private ElapsedTime runtime = new ElapsedTime();
    private EncoderClass encoder = new EncoderClass();
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Servo claw;

    double driveSpeed = 0;
    private ModernRoboticsI2cGyro gyro;

    DcMotor armExtendMotor;
    DcMotor armLiftMotor;
    boolean stage1 = true, stage2 = false, stage3 = false, stage4 = false, stage5 = false, stage6 = false, stage7 = false, stage8 = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables according to the names given in the FTC app
        //MUST BE EXACTLY THE SAME NAME for the motors to be initialized
        frontLeftDrive = hardwareMap.dcMotor.get("front_left_drive");
        frontRightDrive = hardwareMap.dcMotor.get("front_right_drive");
        backLeftDrive = hardwareMap.dcMotor.get("back_left_drive");
        backRightDrive = hardwareMap.dcMotor.get("back_right_drive");
        claw = hardwareMap.servo.get("servo_claw");
        armExtendMotor = hardwareMap.dcMotor.get("arm_extend");         //
        armLiftMotor = hardwareMap.dcMotor.get("arm_lift");
        armExtendMotor.setDirection(DcMotorSimple.Direction.FORWARD);   //
        armLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //ballFlicker = hardwareMap.servo.get("tail");

        //gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyroSens");

        // Set the direction of the motors dedicated to the wheels
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Drive Backwards
        while(stage1 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 1: ", stage1);
            telemetry.addData("Front Left Wheel: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Wheel: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Wheel: ", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Wheel: ", backRightDrive.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(frontLeftDrive, -2200, 0.3);
            encoder.RunToEncoderValue(frontRightDrive, -2200, 0.3);
            encoder.RunToEncoderValue(backLeftDrive, -2200, 0.3);
            encoder.RunToEncoderValue(backRightDrive, -2200, 0.3);
            if(frontLeftDrive.getCurrentPosition() < -2150 && backRightDrive.getCurrentPosition() < -2150)
            {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                stage2 = true;
                stage1 = false;
            }
        }
        //Extend claw
        while(stage2 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 2: ", stage2);
            telemetry.addData("Arm Extension: ", armExtendMotor.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(armLiftMotor, 100, 0.8);
            encoder.RunToEncoderValue(armExtendMotor, 5200, 0.8);
            if(armExtendMotor.getCurrentPosition() > 5150)
            {
                armExtendMotor.setPower(0);
                stage3 = true;
                stage2 = false;
            }
        }
        //Pull in claw
        while(stage3 && opModeIsActive())
        {
            claw.setPosition(0.12);
            //Display stage status
            telemetry.addData("Stage 3: ", stage3);
            telemetry.addData("Arm Extension: ", armExtendMotor.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(armExtendMotor, 0, 0.8);
            if(armExtendMotor.getCurrentPosition() < 10)
            {
                stage4 = true;
                stage3 = false;
            }
        }
        //back up to get into position
        while(stage4 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 4: ", stage4);
            telemetry.addData("Front Left Wheel: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Wheel: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Wheel: ", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Wheel: ", backRightDrive.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(armLiftMotor,5,0.8);
            encoder.RunToEncoderValue(frontLeftDrive, -3600, 0.3);
            encoder.RunToEncoderValue(frontRightDrive, -3600, 0.3);
            encoder.RunToEncoderValue(backLeftDrive, -3600, 0.3);
            encoder.RunToEncoderValue(backRightDrive, -3600, 0.3);
            if(frontLeftDrive.getCurrentPosition() < -3550 && backRightDrive.getCurrentPosition() < -3550)
            {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                stage5 = true;
                stage4 = false;
            }
        }
        //rotate to face the cryptobox
        while(stage5 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 5: ", stage5);
            telemetry.addData("Front Left Wheel: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Wheel: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Wheel: ", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Wheel: ", backRightDrive.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(frontLeftDrive, -6900, 0.3);
            encoder.RunToEncoderValue(frontRightDrive, -1700, 0.3);
            encoder.RunToEncoderValue(backLeftDrive, -6900, 0.3);
            encoder.RunToEncoderValue(backRightDrive, -1800, 0.3);

            if(backLeftDrive.getCurrentPosition() < -6850 && frontRightDrive.getCurrentPosition() < -1650)
            {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                stage6 = true;
                stage5 = false;
            }
        }
        //Extend arm
        while(stage6 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 6: ", stage6);
            telemetry.addData("Arm Extension: ", armExtendMotor.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(armLiftMotor, 100, 0.8);
            encoder.RunToEncoderValue(armExtendMotor, 5200, 0.8);
            if(armExtendMotor.getCurrentPosition() > 5150)
            {
                armExtendMotor.setPower(0);
                claw.setPosition(0.5);
                stage7 = true;
                stage6 = false;
            }
        }
        //Pull in claw
        while(stage7 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 7: ", stage7);
            telemetry.addData("Arm Extension: ", armExtendMotor.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(armExtendMotor, 0, 0.8);
            if(armExtendMotor.getCurrentPosition() < 10)
            {
                stage8 = true;
                stage7 = false;
            }
        }
        //drive forward
        while(stage8 && opModeIsActive())
        {
            //Display stage status
            telemetry.addData("Stage 8: ", stage8);
            telemetry.addData("Front Left Wheel: ", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Wheel: ", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Wheel: ", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Wheel: ", backRightDrive.getCurrentPosition());
            telemetry.update();

            encoder.RunToEncoderValue(frontLeftDrive, -4800, 0.3);
            encoder.RunToEncoderValue(frontRightDrive, 400, 0.3);
            encoder.RunToEncoderValue(backLeftDrive, -4800, 0.3);
            encoder.RunToEncoderValue(backRightDrive, 400, 0.3);

            if(frontLeftDrive.getCurrentPosition() < -4750 && backRightDrive.getCurrentPosition() > 350)
            {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                stage8 = false;
            }
        }
        if(isStopRequested())
        {
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            armExtendMotor.setPower(0);
            armLiftMotor.setPower(0);
        }
    }
}
