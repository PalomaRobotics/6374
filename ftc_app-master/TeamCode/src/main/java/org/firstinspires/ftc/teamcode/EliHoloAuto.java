package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "EvilBots19_20Auto", group = "Auto")

//@Disabled
public class EliHoloAuto extends LinearOpMode {

    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;
    private DistanceSensor USS = null;
    private ElapsedTime elapsedTime = new ElapsedTime();


    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;



    public void runOpMode(){
        RANGE1 = hardwareMap.i2cDevice.get("sonar");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();


        //init
        LFD = hardwareMap.get(DcMotor.class,"FL");
        RFD = hardwareMap.get(DcMotor.class,"FR");
        LBD = hardwareMap.get(DcMotor.class,"BL");
        RBD = hardwareMap.get(DcMotor.class,"BR");
        double[] dirs = {0,0,0,0};
        //end init
        //go left until seconds passed
        //double TimeToWaitTo = elapsedTime.

        waitForStart();
        while(opModeIsActive())
        {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            dirs = new double[] {0,0,0,0};
            if ((range1Cache[0] & 0xFF)>165)
            {


                dirs = HolonomicDrive.RoboMoveXY(1,0);

            }
            if ((range1Cache[0] & 0xFF)<135)
            {
                dirs = HolonomicDrive.RoboMoveXY(-1,0);
            }

            LFD.setPower(dirs[0]);
            RFD.setPower(dirs[1]);
            LBD.setPower(dirs[2]);
            RBD.setPower(dirs[3]);


            telemetry.addData("distance", range1Cache[0] & 0xFF);
            telemetry.addData("number", 8);
        }
    }
}
