package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Array;

public class HolonomicDrive {
    private static DcMotor LFD = null;
    private static DcMotor RFD = null;
    private static DcMotor LBD = null;
    private static DcMotor RBD = null;
    private static float fr;
    private static float fl;
    private static float br;
    private static float bl;

    public static float[] RoboMoveDir(float Degrees)
    {
        float d = Degrees % 360;
        fr=(((d+45)%360)/90)-2;
        if (fr>1)
        {
            fr=fr-2*(fr-1);
        }
        else if (fr<-1)
        {
            fr=fr-2*(fr+1);
        }


        fl=(((d+315)%360)/90)-2;
        if (fl>1)
        {
            fl=fl-2*(fl-1);
        }
        else if (fl<-1)
        {
            fl=fl-2*(fl+1);
        }


        br=(((d+135)%360)/90)-2;
        if (br>1)
        {
            br=br-2*(br-1);
        }
        else if (br<-1)
        {
            br=br-2*(br+1);
        }


        bl=(((d+255)%360)/90)-2;
        if (bl>1)
        {
            bl=bl-2*(bl-1);
        }
        else if (bl<-1) {
            bl = bl - 2 * (bl + 1);
        }
        float[] ar = {fl,fr,bl,br};
        for(int i=0;i<ar.length;i++)
        {
            if(!(ar[i]>-1 && ar[i]<1))
            {
                ar[i]=0;
            }

        }

        return ar;



    }


    public static float XYtoDeg(float x, float y)
    {
        double dr = Math.atan(-y/x);
        double dd = dr*180/Math.PI-90;
        if(x<0)
        {
            dd+=180;
        }
        return (float) dd;
    }

}
