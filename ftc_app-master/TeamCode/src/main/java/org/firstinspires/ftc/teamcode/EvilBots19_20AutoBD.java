package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Build or Blue Depot",group = "Auto")
public class EvilBots19_20AutoBD extends LinearOpMode {

    private DcMotor LFD = null;
    private DcMotor RFD = null;
    private DcMotor LBD = null;
    private DcMotor RBD = null;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public void runOpMode() {
        LFD = hardwareMap.get(DcMotor.class,"FL");
        RFD = hardwareMap.get(DcMotor.class,"FR");
        LBD = hardwareMap.get(DcMotor.class,"BL");
        RBD = hardwareMap.get(DcMotor.class,"BR");
        waitForStart();
        double dirs[] = HolonomicDrive.RoboMoveXY(-1,0);
        double timeToGoTo = elapsedTime.milliseconds() + 1500;
        while(elapsedTime.milliseconds()<timeToGoTo){
            LFD.setPower(dirs[0]);
            RFD.setPower(dirs[1]);
            LBD.setPower(dirs[2]);
            RBD.setPower(dirs[3]);
        }
        LFD.setPower(0);
        RFD.setPower(0);
        LBD.setPower(0);
        RBD.setPower(0);
    }
}
