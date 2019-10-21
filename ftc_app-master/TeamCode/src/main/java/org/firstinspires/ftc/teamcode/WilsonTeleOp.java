package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LegacyModulePortDevice;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

//used by i2c sensors (Modern Robotics Range Ultrasonic Sensor)
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


/**
 * Created by robotics on 11/7/2016.
 */
@TeleOp (name = "WilsonTeleOp", group = "TeleOp")
@Disabled
public class WilsonTeleOp extends OpMode
{
//test
	//Class-level variables go here

	//Motors Variables:
	//These names need to match the ones in the phone's device configuration

	private DcMotor fl; //class-level variable for the left motor.
	private DcMotor fr; //class-level variable for the right motor.
	private DcMotor bl; //class-level variable for the left motor.
	private DcMotor br; //class-level variable for the right motor.

	//Sensor variables:
	//These variables create objects based on the imported classes
	//These variables do NOT correspond to the names in the phone's device configuration

	//i2c declaraions
	ModernRoboticsI2cGyro gyro;   //Don't forget to import the proper library: import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

	GyroClass gyroObject; //create a variable capable of holding an object based off the GyroClass class

	public WilsonTeleOp() {

	}//leave empty

	public final void sleep(long milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
	}

	public final void idle() {
		// Otherwise, yield back our thread scheduling quantum and give other threads at
		// our priority level a chance to run
		Thread.yield();
	}


	@Override
	public void init()
	{

		//Code to initialize all variables and whatnot goes here. This method runs once when the code first starts

		//Motors
		fl = hardwareMap.dcMotor.get("FL"); //Find a motor on the robot named left
		fr = hardwareMap.dcMotor.get("FR"); //Find a motor on the robot named right
		bl = hardwareMap.dcMotor.get("BL"); //Find a motor on the robot named left
		br = hardwareMap.dcMotor.get("BR"); //Find a motor on the robot named right
		fr.setDirection(DcMotor.Direction.REVERSE); //somtimes you may need to reverse the direction of a motor to insure positive number run forward and negative numbers run backward
		br.setDirection(DcMotor.Direction.REVERSE); //somtimes you may need to reverse the direction of a motor to insure positive number run forward and negative numbers run backward

		//Sensors
		//Gyro-specific variables
		gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("GYRO1"); //THIS IS AN I2C DEVICE. Find a Gyro Sensor on the robot named gyroSens. gyroSens is the name that appears in the phones configuration
		/*
		/////////////////////////Gyro Initialization
		//You cannot use this gyro initialization code if you are using the GyroClass since the GyroClass has it built in already
		gyro.calibrate(); //calibrate the gyro. DON'T MOVE THE ROBOT OR SENSOR UNTIL THIS IS DONE!

		// This loop ties up the robot until gyro calibration is complete
		while (gyro.isCalibrating())  {
			sleep(50);
			idle();
		}

		gyro.resetZAxisIntegrator(); //reset the heading. The sensor only returns a heading for the Z axis
		/////////////////////////
		*/

		//GyroClass
		gyroObject = new GyroClass(gyro, fl, fr, 0.25); //Instantiate the variable we made earlier by storing an object in it. Initialize the object using the motos and gyro object we created above

		super.msStuckDetectLoop = 30000;

	}//initialize

	/*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */



	boolean doneonce = false;
	@Override
	public void loop()
	{

		//MOVE MOTORS BASED ON JOYSTICK VALUES///////////////////////////////////////////////////////////
		//left.setPower(gamepad1.left_stick_y); //gamepad1.right_stick_y
		//right.setPower(gamepad1.right_stick_y); //gamepad1.right_stick_y
		/////////////////////////////////////////////////////////////////////////////////

		//GYRO SENSOR EXAMPLE////////////////////////////////////////////////////////////
		/*
		//These variable can be used to read the various values from the gyro sensor.
		//the heading
		int xVal, yVal, zVal = 0;     // Gyro rate Values for each axis
		int heading = 0;              // Gyro integrated heading
		int angleZ = 0;
		boolean lastResetState = false;
		boolean curResetState  = false;

		xVal = gyro.rawX(); // get the x, y, and z values (rate of change of angle).
		yVal = gyro.rawY();
		zVal = gyro.rawZ();

		heading = gyro.getHeading(); //get the heading info. The Modern Robotics' gyro sensor keeps track of the current heading for the Z axis only.
		angleZ  = gyro.getIntegratedZValue();

		telemetry.addData("--", heading);
		telemetry.addData("0", "Heading %03d", heading);
		telemetry.addData("1", "Int. Ang. %03d", angleZ);
		telemetry.addData("2", "X av. %03d", xVal);
		telemetry.addData("3", "Y av. %03d", yVal);
		telemetry.addData("4", "Z av. %03d", zVal);
		*/
		/////////////////////////////////////////////////////////////////////////////////


		if(!doneonce) {
			gyroObject.Turn(270);
			doneonce = true;
		}
		gyroObject.StraightLineUpdate(); //You can use the GyroClass to drive the robot in a straight line using the gyro to maintain your heading. Calling update in the loop keeps updating the robots position. You can stop calling update and stop the motors at anytime to go somewhere else
		//gyroObject.ResetHeading(); //calling ResetHeading resets the gyro heading to zero. For example, you could use Update() to drive straight for awhile, then change direction, reset the heading, and drive straight again. If you don't reset the heading, the robot would turn itself back to zero degrees again


		telemetry.update();


	}//end main loop


	@Override
	public void stop() {

	}

	/*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);
		if (index < 0) {
			index = -index;
		} else if (index > 16) {
			index = 16;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}

}

