package org.firstinspires.ftc.teamcode.image;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.content.res.Resources;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

public class LayoutModifier
{
    private Activity act = null;

    private Resources rsrcs;
    private String pName;

    public LayoutModifier(HardwareMap hardwareMap)
    {
        act   = (Activity) hardwareMap.appContext;
        rsrcs = hardwareMap.appContext.getResources();
        pName = hardwareMap.appContext.getPackageName();
    }

    private int getId(String vName)
    {
        return rsrcs.getIdentifier(vName, "id", pName);
    }

    public void configureViewLayout()
    {
        act.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        class CameraRunnable implements Runnable
        {
            public void run()
            {
                RobotLog.dd("SJH", "Configuring Layout");

                //ViewGroup es = (ViewGroup) act.findViewById(R.id.entire_screen);
                ViewGroup es = act.findViewById(getId("entire_screen"));
                ViewGroup tb = act.findViewById(getId("top_bar"));
                ViewGroup ih = act.findViewById(getId("included_header"));
                if(tb != null) es.removeView(tb);
                if(ih != null) es.removeView(ih);

                ViewGroup rl = act.findViewById(getId("RelativeLayout"));
                View nc = act.findViewById(getId("textNetworkConnectionStatus"));
                View rs = act.findViewById(getId("textRobotStatus"));
                View g1 = act.findViewById(getId("textGamepad1"));
                View g2 = act.findViewById(getId("textGamepad2"));
                View wv = act.findViewById(getId("webViewBlocksRuntime"));
                if(nc != null) rl.removeView(nc);
                if(rs != null) rl.removeView(rs);
                if(g1 != null) rl.removeView(g1);
                if(g2 != null) rl.removeView(g2);
                if(wv != null) rl.removeView(wv);

                rl.setPadding(0, 0, 0, 0);

                //TextView et = act.findViewById(getId("textErrorMessage"));
                //et.setTextColor(0xFFFF00);
            }
        }

        CameraRunnable mr = new CameraRunnable();
        act.runOnUiThread(mr);
    }
}
