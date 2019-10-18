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
@TeleOp(name = "EliHOLOTeleOp", group = "TeleOp")

public class HoloTestTeleOp extends OpMode {
    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;

    public void init(){
      LFD = hardwareMap.get(DcMotor.class,"DLeftF");
      RFD = hardwareMap.get(DcMotor.class,"DRightF");
      LBD = hardwareMap.get(DcMotor.class,"DLeftB");
      RBD = hardwareMap.get(DcMotor.class,"DRightB");
    }
    public void loop() {
        double inx = gamepad1.left_stick_x;
        double iny = gamepad1.left_stick_y;
        double dir = Math.asin(inx) * Math.acos(iny);

    }


}
