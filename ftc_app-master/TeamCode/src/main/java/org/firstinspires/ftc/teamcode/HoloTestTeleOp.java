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
        double inx = (double)gamepad1.left_stick_x;
        double iny = (double)gamepad1.left_stick_y;
        float[] dirs = HolonomicDrive.RoboMoveDir(HolonomicDrive.XYtoDeg((float)inx,(float)iny));
        LFD.setPower(dirs[0]);
        RFD.setPower(dirs[1]);
        LBD.setPower(dirs[2]);
        RBD.setPower(dirs[3]);
        telemetry.addData("Stick X", gamepad1.left_stick_x);
        telemetry.addData("Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Dirctions", ((Math.round(dirs[0]*100)/100)));
        telemetry.update();
    }


}
