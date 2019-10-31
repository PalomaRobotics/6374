package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
@TeleOp(name = "EliHOLOTeleOp", group = "TeleOp")

public class HoloTestTeleOp extends OpMode {
    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;

    public void init(){
      LFD = hardwareMap.get(DcMotor.class,"FL");
      RFD = hardwareMap.get(DcMotor.class,"FR");
      LBD = hardwareMap.get(DcMotor.class,"BL");
      RBD = hardwareMap.get(DcMotor.class,"BR");
    }
    public void loop() {

        //get input
        double inx = (double)gamepad1.left_stick_x;
        double iny = (double)gamepad1.left_stick_y;
        double inRotation = (double)gamepad1.right_stick_x;

        //calculate powers
        double[] dirs = HolonomicDrive.RoboMoveXY(inx,iny);
        double[] rotToAdd = HolonomicDrive.RoboRotate(inRotation);

        //average values
        for(int i=0;i<rotToAdd.length;i++)
        {
            dirs[i]=(dirs[i]+rotToAdd[i])/2;
        }
        //decreases power if fine control btn is presses
        if(gamepad1.b)
        {
            for(int i=0;i<dirs.length;i++)
            {
                dirs[i]=dirs[i]/4;
            }
        }


        //set power for motors
        LFD.setPower(dirs[0]);
        RFD.setPower(dirs[1]);
        LBD.setPower(dirs[2]);
        RBD.setPower(dirs[3]);

        //telemetry
        telemetry.addData("Stick X", gamepad1.left_stick_x);
        telemetry.addData("Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Powers", (String.valueOf(Math.round(dirs[0]*100)/100)+String.valueOf(Math.round(dirs[1]*100)/100)+String.valueOf(Math.round(dirs[2]*100)/100)+String.valueOf(Math.round(dirs[3]*100)/100)));
        telemetry.addData("Direction",(HolonomicDrive.XYtoDeg((float)inx,(float)iny)));
        telemetry.update();
    }


}
