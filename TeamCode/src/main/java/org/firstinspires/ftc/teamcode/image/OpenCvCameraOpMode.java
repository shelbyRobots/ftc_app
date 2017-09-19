package org.firstinspires.ftc.teamcode.image;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.content.res.Resources;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;

@SuppressWarnings("unused")
public abstract class OpenCvCameraOpMode extends LinearOpMode
                                         implements CameraBridgeViewBase.CvCameraViewListener2
{
    protected boolean newImage = false;
    protected ImageProcessor imgProc = null;
    private static JavaCameraView openCVCamera = null;

    private Resources rsrcs;
    private String pName;

    protected boolean flipImage       = false;
    private boolean configureLayout   = true;
    private boolean addJavaCameraView = true;

    private int width;
    private int height;
    private boolean initialized = false;

    int getWidth()  {return  width;}
    int getHeight() {return height;}
    boolean isInitialized() {
        return initialized;
    }

//    public void setCamera() {
//        if (openCVCamera == null)
//            return;
//        openCVCamera.disableView();
//        if (initialized) openCVCamera.disconnectCamera();
//        openCVCamera.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
//        if (initialized)
//            if (!openCVCamera.connectCamera(width, height))
//                RobotLog.ee("SJH", Could not initialize camera!\r\n" +
//                              "This may occur because the OpenCV Manager is not installed,\r\n" +
//                              "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
//                              "or because another app is currently locking it.");
//        openCVCamera.enableView();
//    }
//
//    public Size setFrameSize(Size frameSize) {
//        if (openCVCamera == null)
//            return null;
//
//        openCVCamera.disableView();
//        if (initialized) openCVCamera.disconnectCamera();
//        openCVCamera.setMaxFrameSize((int) frameSize.width, (int) frameSize.height);
//        if (initialized)
//            if (!openCVCamera.connectCamera((int) frameSize.width, (int) frameSize.height))
//                RobotLog.ee("SJH", "Could not initialize camera!\r\n" +
//                              "This may occur because the OpenCV Manager is not installed,\r\n" +
//                              "CAMERA permission is not allowed in AndroidManifest.xml,\r\n" +
//                              "or because another app is currently locking it.");
//        openCVCamera.enableView();
//
//        width = openCVCamera.getMeasuredWidth();
//        height = openCVCamera.getMeasuredHeight();
//        if (width == 0 || height == 0) {
//            RobotLog.ee("SJH", "OpenCV Camera failed to initialize width and height properties on startup.\r\n" +

//                                       "This is generally okay, but if you use width or height during init() you may\r\n" +
//                                       "run into a problem.");
//        }
//
//        return new Size(width, height);
//    }

    private int getId(String vName)
    {
        return rsrcs.getIdentifier(vName, "id", pName);
    }

    private void setupCameraView()
    {
        rsrcs = hardwareMap.appContext.getResources();
        pName = hardwareMap.appContext.getPackageName();

        final Activity act = (Activity) hardwareMap.appContext;
        act.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        class CameraRunnable implements Runnable
        {
            private JavaCameraView jcv = null;
            private JavaCameraView getJcv() { return jcv; }

            public void run() {
                RobotLog.dd("SJH", "Configuring Layout");

                if(configureLayout)
                {
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

                if(addJavaCameraView)
                {
                    //Add a JavaCameraView child to cameraMonitorView
                    ViewGroup vg = act.findViewById(getId("cameraMonitorViewId"));

                    jcv = new JavaCameraView(act, CameraBridgeViewBase.CAMERA_ID_BACK);
                    jcv.setMaxFrameSize(480, 320);

                    ViewGroup.LayoutParams vglop =
                            new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                                                              ViewGroup.LayoutParams.MATCH_PARENT);
                    jcv.setLayoutParams(vglop);
                    vg.addView(jcv, 0);
                }

                initialized = true;
            }
        }

        CameraRunnable mr = new CameraRunnable();
        act.runOnUiThread(mr);

        JavaCameraView jcv = null;
        ElapsedTime jtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(jcv == null && jtimer.milliseconds() < 5000)
        {
            jcv = mr.getJcv();
        }

        if(jcv == null)
        {
            RobotLog.ww("SJH", "JavaCameraView created");
            return;
        }

        openCVCamera = jcv;
        openCVCamera.setVisibility(CameraBridgeViewBase.VISIBLE);
        openCVCamera.setCvCameraViewListener(this);

        width = openCVCamera.getMeasuredWidth();
        height = openCVCamera.getMeasuredHeight();
        RobotLog.ii("SJH", "CAMERA %d x %d", width, height);
    }

    protected void initOpenCv(boolean configureLayout, boolean addJavaCameraView)
    {
        this.configureLayout   = configureLayout;
        this.addJavaCameraView = addJavaCameraView;
        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        RobotLog.dd("SJH", "OpenCvLoad: SUCCESS - setting up JCV");
                        setupCameraView();
                        RobotLog.dd("SJH", "OpenCvLoad: SUCCESS - enabling view");
                        openCVCamera.enableView();
                        RobotLog.dd("SJH", "OpenCvLoad: SUCCESS - called enableView");
                    }
                    break;
                    default: {
                        RobotLog.dd("SJH", "OpenCvLoad: FAILURE");
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        if (!OpenCVLoader.initDebug())
        {
            RobotLog.dd("SJH", "OpenCvLoad: Could not load from app - trying OpenCV manager");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,
                    hardwareMap.appContext,
                    mLoaderCallback);
        } else
        {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    protected void initOpenCv()
    {
        this.initOpenCv(true, true);
    }

    protected void cleanupCamera()
    {
        if (openCVCamera != null)
        {
            RobotLog.dd("SJH", "cleanupCamera");
            openCVCamera.disableView();
            openCVCamera.disconnectCamera();
        }
    }

    protected void setImageProcessor(ImageProcessor imgProc)
    {
        this.imgProc = imgProc;
    }

    protected ImageProcessor getImageProcessor() { return imgProc; }

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        RobotLog.ii("SJH", "CAMERA VIEW STARTED %4dx%4d", width, height);
    }

    @Override
    public void onCameraViewStopped()
    {
        RobotLog.ii("SJH", "CAMERA VIEW STOPPED");
        if(imgProc != null)
        {
            imgProc.stopSensing();
        }

        if (openCVCamera != null)
        {
            openCVCamera.disableView();
        }
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        Mat rgb = inputFrame.rgba();
        newImage = true;

        Mat rtrnImage = rgb;

        if(imgProc != null)
        {
            if(flipImage)
            {
                Mat flip = rgb.clone();
                Core.flip(rgb, flip, 1);
                imgProc.setImage(flip);
            }
            else
            {
                imgProc.setImage( rgb );
            }

            rtrnImage = imgProc.draw();
        }

        return rtrnImage;
    }
}
