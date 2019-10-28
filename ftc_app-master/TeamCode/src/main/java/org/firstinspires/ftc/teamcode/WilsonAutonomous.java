/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

@Autonomous (name = "WilsonAutonomous", group = "Autonomous")
public class WilsonAutonomous extends LinearOpMode {

	DcMotor FL;
	DcMotor FR;
	DcMotor BL;
	DcMotor BR;

	ModernRoboticsI2cGyro gyro;   //Don't forget to import the proper library: import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

	GyroClass gyroObject; //create a variable capable of holding an object based off the GyroClass class



	@Override
	public void runOpMode() throws InterruptedException {
		FL = hardwareMap.dcMotor.get("FL");
		BL = hardwareMap.dcMotor.get("BL");
		FL.setDirection(DcMotor.Direction.REVERSE);
		BL.setDirection(DcMotor.Direction.REVERSE);


		FR = hardwareMap.dcMotor.get("FR");
		BR = hardwareMap.dcMotor.get("BR");

		gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyroSens"); //THIS IS AN I2C DEVICE. Find a Gyro Sensor on the robot named gyroSens. gyroSens is the name that appears in the phones configuration

		gyroObject = new GyroClass(gyro,FL,FR,BL,BR,0.30,telemetry); //instantiate the gyroClass object

		gyroObject.ResetHeading();

		waitForStart();
		//telemetry.addData("Time To Travel: ", "" + timer);

		while(true)
		{
			gyroObject.StraightLineUpdate();
		}


	}


}
