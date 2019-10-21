package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

//used by i2c sensors (Modern Robotics Range Ultrasonic Sensor)


/**
 * Created by robotics on 11/7/2016.
 */
@TeleOp (name = "WilsonCodeExamples", group = "TeleOp")
@Disabled
public class WilsonCodeExamples extends OpMode
{

	//Class-level variables go here

	//Motors Variables:
	//These names need to match the ones in the phone's device configuration
	private DcMotorController MC1; //class-level variable for the motor controller
	private DcMotor left; //class-level variable for the left motor.
	private DcMotor right; //class-level variable for the right motor.

	//Sensor variables:
	//These variables create objects based on the imported classes
	//These variables do NOT correspond to the names in the phone's device configuration
	TouchSensor touchSensor; //Don't forget to import the proper library: import com.qualcomm.robotcore.hardware.TouchSensor;
	IrSeekerSensor irSeeker; //Don't forget to import the proper library: import com.qualcomm.robotcore.hardware.IrSeekerSensor;
	OpticalDistanceSensor odsSensor;  //Don't forget to import the proper library: import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

	UltrasonicSensor mySonar; //Don't forget to import the proper library: import com.qualcomm.robotcore.hardware.UltrasonicSensor;
	//You may or may not also need to import the library for the legacy module?:
	//import com.qualcomm.robotcore.hardware.LegacyModule;
	//import com.qualcomm.robotcore.hardware.LegacyModulePortDevice;

	//i2c declaraions (Modern Robotics Range Ultrasonic Sensor)
	byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
	I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
	public static final int RANGE1_REG_START = 0x04; //Register to start reading
	public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
	public I2cDevice RANGE1;
	public I2cDeviceSynch RANGE1Reader;


	ModernRoboticsI2cGyro gyro;   //Don't forget to import the proper library: import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

	GyroClass gyroObject; //create a variable capable of holding an object based off the GyroClass class

	public WilsonCodeExamples() {

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
		MC1 = hardwareMap.dcMotorController.get("MC1"); //Find a motor controller on the robot named MC1. Note, this must match the name in the phone configuration
		left = hardwareMap.dcMotor.get("left"); //Find a motor on the robot named left
		right = hardwareMap.dcMotor.get("right"); //Find a motor on the robot named right
		right.setDirection(DcMotor.Direction.REVERSE); //somtimes you may need to reverse the direction of a motor to insure positive number run forward and negative numbers run backward

		//Sensors
		touchSensor = hardwareMap.touchSensor.get("touch1"); //THIS IS A DIGITAL DEVICE. Find a touchsensor on the robot named touch1. touch1 is the name that appears in the phones configuration
		irSeeker = hardwareMap.irSeekerSensor.get("ir"); //THIS IS A I2C DEVICE. Find a IR Sensor on the robot named ir. ir is the name that appears in the phones configuration
		odsSensor = hardwareMap.opticalDistanceSensor.get("OpticalSens"); //THIS IS AN ANALOG DEVICE. Find a ODS Sensor on the robot named OpticalSens. OpticalSens is the name that appears in the phones configuration
		mySonar = hardwareMap.ultrasonicSensor.get("sonar"); //THIS DEVICE MUST BE ATTACHED TO THE LEGACY MODULE. Find a Ultrasonic Sensor on the robot named sonar. sonar is the name that appears in the phones configuration (ONLY WORKS WITH PORTS S5 AND S4)


		//Modern Robotics Range Ultrasonic Sensor
		RANGE1 = hardwareMap.i2cDevice.get("range");
		RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
		RANGE1Reader.engage();


		//Gyro-specific variables
		gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyroSens"); //THIS IS AN I2C DEVICE. Find a Gyro Sensor on the robot named gyroSens. gyroSens is the name that appears in the phones configuration
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
		gyroObject = new GyroClass(gyro, left, right, 0.25); //Instantate the variable we made earlier by storing an object in it. Initialize the object using the motos and gyro object we created above

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
		//java.lang.Object.OpModeManagerImpl.stopMonitoring();

		//This method runs over and over after the robot initializes

		//TOUCH SENSOR EXAMPLE///////////////////////////////////////////////////////////
		if (touchSensor.isPressed()) {
			telemetry.addData("Touch", "Is Pressed");
		}
		else {
			telemetry.addData("Touch", "Is Not Pressed");
		}
		/////////////////////////////////////////////////////////////////////////////////

		//IR SENSOR EXAMPLE///////////////////////////////////////////////////////////
		if (irSeeker.signalDetected())
		{
			// Display angle and strength
			telemetry.addData("Angle",    irSeeker.getAngle()); //positive number indicate becon is to the right of robot. Negative number indicate beacon is to the left
			telemetry.addData("Strength", irSeeker.getStrength()); //0.6 is really close, .06 is far
		}
		else
		{
			// Display loss of signal
			telemetry.addData("Seeker", "Signal Lost");
		}
		/////////////////////////////////////////////////////////////////////////////////

		//OPTICAL DISTANCE SENSOR EXAMPLE///////////////////////////////////////////////////////////
		telemetry.addData("Raw",    odsSensor.getRawLightDetected()); //Float numbers approaching 1 (over 1 if very close) indicate lighter color. Numbers approaching 0 indicate darker colors.
		telemetry.addData("Normal", odsSensor.getLightDetected());
		/////////////////////////////////////////////////////////////////////////////////

		//ULTRASONIC SENSOR EXAMPLE///////////////////////////////////////////////////////////
		telemetry.addData("Sonar:", mySonar.getUltrasonicLevel()); //integers approaching 0 are close. Numbers approaching 255 are far away. Max distance is a little under 8 feet.
		/////////////////////////////////////////////////////////////////////////////////

		//Modern Robotics Ultrasonic Range Finder Example///////////////////////////////////////////////////////////////////////////////
		range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

		telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
		telemetry.addData("ODS", range1Cache[1] & 0xFF);
		telemetry.update();
		/////////////////////////////////////////////////////////////////////////////////

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

		//ENCODER EXAMPLE////////////////////////////////////////////////////////////////
		if(gamepad1.a) //if you press A button...
		{
			//this runs the motor 180 degrees and suspends code execution until it's done
			EncoderClass.RunToEncoderDegree(left, EncoderClass.MotorType.NeveRest40,180,0.25,false);

			//This runs the motor 180 degrees. Code continues to run while this is happening
			EncoderClass.RunToEncoderDegreeAsync(left, EncoderClass.MotorType.NeveRest40,180,0.25,false);
		}
		/////////////////////////////////////////////////////////////////////////////////

		if(!doneonce) {
			gyroObject.Turn(270);
			doneonce = true;
		}
		gyroObject.StraightLineUpdate(); //You can use the GyroClass to drive the robot in a straight line using the gyro to maintain your heading. Calling update in the loop keeps updating the robots position. You can stop calling update and stop the motors at anytime to go somewhere else
		//gyroObject.ResetHeading(); //calling ResetHeading resets the gyro heading to zero. For example, you could use Update() to drive straight for awhile, then change direction, reset the heading, and drive straight again. If you don't reset the heading, the robot would turn itself back to zero degrees again


		telemetry.update();
// *57=trace

/*
		State booYeah;

		booYeah = State.LEFTRUN;

		switch (booYeah) {
			case LEFTRUN:
				left.setPower(1.0);
				break;
			case RIGHTRUN:
				right.setPower(1.0); //gamepad1.right_stick_y
				break;
		}
*/


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

