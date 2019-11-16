package org.firstinspires.ftc.teamcode;

import java.sql.Array;

public class HolonomicDrive {
    private static double fr;
    private static double fl;
    private static double br;
    private static double bl;

    public static double[] RoboRotate(double power)
    {
        power = -power;
        double[] ar = {power,power,power,power};
       return ar;
    };

    public static double[] RoboMoveXY(double x,double y)
    {
        return RoboMoveDir(XYtoDeg(x,y));
    }
    public static double[] RoboMoveDir(double Degrees)
    {
        double d = Degrees % 360;
        fr=(((d+315)%360)/90)-2;
        if (fr>1)
        {
            fr=fr-2*(fr-1);
        }
        else if (fr<-1)
        {
            fr=fr-2*(fr+1);
        }


        fl=(((d+45)%360)/90)-2;
        if (fl>1)
        {
            fl=fl-2*(fl-1);
        }
        else if (fl<-1)
        {
            fl=fl-2*(fl+1);
        }


        br=(((d+225)%360)/90)-2;
        if (br>1)
        {
            br=br-2*(br-1);
        }
        else if (br<-1)
        {
            br=br-2*(br+1);
        }


        bl=(((d+135)%360)/90)-2;
        if (bl>1)
        {
            bl=bl-2*(bl-1);
        }
        else if (bl<-1) {
            bl = bl - 2 * (bl + 1);
        }
        double[] ar = {br,-fr,fr,-br};
        for(int i=0;i<ar.length;i++)
        {
            if(!(ar[i]>=-1 && ar[i]<1))
            {
                ar[i]=0;
            }
            /*if(ar[i]>=0) {
                if (ar[i] < 0.75 && ar[i] > 0) {
                    ar[i] = ar[i] / 3;
                } else if (ar[i] > 0.75) {
                    ar[i] = (ar[i] * 3) - 2;
                } else
                    ar[i] = 0;
            }else{
                ar[i]=Math.abs(ar[i]);
                if (ar[i] < 0.75 && ar[i] > 0) {
                    ar[i] = ar[i] / 3;
                } else if (ar[i] > 0.75) {
                    ar[i] = (ar[i] * 3) - 2;
                } else{
                    ar[i] = 0;}
                ar[i]=ar[i]*-1;

            }*/


        }

        return ar;



    }


    public static double XYtoDeg(double x, double y)
    {
        double dr = Math.atan(y/x);
        double dd = dr*180/Math.PI-90;
        if(x<0)
        {
            dd+=180;
        }

        return dd;
    }

}
