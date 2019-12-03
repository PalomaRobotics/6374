package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
@Autonomous(name = "Blue Build or Red Depot",group = "Auto")
public class EvilBots19_20AutoRB extends LinearOpMode {
    //range setup
    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;



    //other setup
    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;
    private GyroSensor gyro;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private void Rotate(double dir,double spd)
    {
        while(gyro.getHeading()>(dir-2)%360||gyro.getHeading()<94){
        if(gyro.getHeading()>90){
            HolonomicDrive.RoboRotate(0.1);
        }
        if(gyro.getHeading()<90){
            HolonomicDrive.RoboRotate(0.1);
        }
        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);
    }
    }

    public void runOpMode() {
        //range
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        LFD = hardwareMap.get(DcMotor.class,"FL");
        RFD = hardwareMap.get(DcMotor.class,"FR");
        LBD = hardwareMap.get(DcMotor.class,"BL");
        RBD = hardwareMap.get(DcMotor.class,"BR");
        gyro = hardwareMap.get(GyroSensor.class,"GYRO");
        waitForStart();
        double dirs[] = HolonomicDrive.RoboMoveXY(1,0);
        double timeToGoTo = elapsedTime.milliseconds() + 1500;
        while((range1Cache[0] & 0xFF)> 60){
            LFD.setPower(dirs[0]);
            RFD.setPower(dirs[1]);
            LBD.setPower(dirs[2]);
            RBD.setPower(dirs[3]);
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        }
        gyro.resetZAxisIntegrator();
        while(gyro.getHeading()>86||gyro.getHeading()<94){
            if(gyro.getHeading()>90){
                HolonomicDrive.RoboRotate(0.1);
            }
            if(gyro.getHeading()<90){
                HolonomicDrive.RoboRotate(0.1);
            }
            LFD.setPower(0);
            RFD.setPower(0);
            LBD.setPower(0);
            RBD.setPower(0);
        }
        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);
    }//end opmode
}//end class
