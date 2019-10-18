package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//used by i2c sensors:
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;


@Autonomous(name = "WilsonSandbox", group = "Auto")
//@Disabled
public class WilsonSandbox extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;


    //i2c declaraions (ultrasonic sensor)
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;




    @Override
    public void runOpMode() {

        //ultrasonic init:
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "motor1");

        //colorSensor=hardwareMap.get(ModernRoboticsI2cColorSensor.class,"color");
        //range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        /*
        new Thread(new Runnable() {
            public void run() {
                EncoderClass.RunToEncoderDegree(leftDrive,180,0.25);
            }
        }).start();
         */

        //EncoderClass.RunToEncoderDegreeAsync(leftDrive, EncoderClass.MotorType.NeveRest40,180,0.25,false);
        //EncoderClass.RunToEncoderDegree(leftDrive, EncoderClass.MotorType.NeveRest40,180,0.25,false);

        while (opModeIsActive()) {
            //telemetry.addData("encoder-fwd", leftDrive.getCurrentPosition() + "  busy=" + leftDrive.isBusy());
            //telemetry.update();
            //idle();
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("ODS", range1Cache[1] & 0xFF);
            telemetry.update();
            idle();
        }
        /*
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(5000);
        leftDrive.setPower(0.25);

        while (opModeIsActive() && leftDrive.isBusy())
        {
            telemetry.addData("encoder-fwd", leftDrive.getCurrentPosition() + "  busy=" + leftDrive.isBusy());
            telemetry.update();
            idle();
        }
        leftDrive.setPower(0.0);
        */

    }
}
