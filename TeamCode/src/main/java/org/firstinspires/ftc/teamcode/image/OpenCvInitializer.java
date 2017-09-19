package org.firstinspires.ftc.teamcode.image;


import android.app.Activity;
import android.content.res.Resources;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.hardware.HardwareMap;
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
public class OpenCvInitializer implements CameraBridgeViewBase.CvCameraViewListener2
{
    private boolean newImage = false;
    private ImageProcessor imgProc = null;
    private static JavaCameraView openCVCamera = null;

    private HardwareMap hardwareMap;
    private Activity act = null;

    private Resources rsrcs;
    private String pName;

    private boolean flipImage           = false;
    private boolean configureLayout     = true;
    private boolean addJavaCameraView   = true;
    private static boolean initialized  = false;
    private static boolean initializing = false;

    private int width;
    private int height;

    int getWidth()  {return  width;}
    int getHeight() {return height;}
    public  static boolean isInitializing() {return initializing;}
    private static boolean isInitialized()  {return initialized;}

    public OpenCvInitializer(boolean configureLayout)
    {
        this(null, configureLayout, false);
    }

    public OpenCvInitializer(HardwareMap hardwareMap,
                             boolean configureLayout,
                             boolean addJavaCameraView)
    {
        this.hardwareMap = hardwareMap;
        this.configureLayout   = configureLayout;
        this.addJavaCameraView = addJavaCameraView;

        if(hardwareMap != null)
        {
            act = (Activity) hardwareMap.appContext;
            rsrcs = hardwareMap.appContext.getResources();
            pName = hardwareMap.appContext.getPackageName();
        }

        initOpenCv();
    }

    private int getId(String vName)
    {
        return rsrcs.getIdentifier(vName, "id", pName);
    }

    private void configureViewLayout()
    {
        if(!configureLayout || hardwareMap == null) return;
        RobotLog.dd("SJH", "OpenCvLoad: configuring layout");
        LayoutModifier lmod = new LayoutModifier(hardwareMap);
        lmod.configureViewLayout();
    }

    private void setupCameraView()
    {
        if(!addJavaCameraView || hardwareMap == null) return;

        RobotLog.dd("SJH", "OpenCvLoad: setting up JCV");
        class CameraRunnable implements Runnable
        {
            private JavaCameraView jcv = null;
            private JavaCameraView getJcv() { return jcv; }

            public void run() {
                RobotLog.dd("SJH", "Configuring Layout");

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

        RobotLog.dd("SJH", "OpenCvLoad: enabling view");
        openCVCamera.enableView();
        RobotLog.dd("SJH", "OpenCvLoad: called enableView");
    }

    private void setup()
    {
        configureViewLayout();
        setupCameraView();
    }

    private void initOpenCv()
    {
        if(initialized || initializing)
        {
            RobotLog.dd("SJH", "OpenCV already initialized");
            return;
        }

        initializing = true;

        BaseLoaderCallback mLoaderCallback = null;

        if(hardwareMap != null)
        {
            mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext)
            {
                @Override
                public void onManagerConnected(int status)
                {
                    switch (status)
                    {
                        case LoaderCallbackInterface.SUCCESS:
                        {
                            setup();
                        }
                        break;
                        default:
                        {
                            RobotLog.dd("SJH", "OpenCvLoad: FAILURE");
                            super.onManagerConnected(status);
                        }
                        break;
                    }
                }
            };
        }

        //Try loading ocv libraries from within app pkg
        if (!OpenCVLoader.initDebug())
        {
            RobotLog.dd("SJH", "OpenCvLoad: Could not load from app - trying OpenCV manager");
            //Try loading using OpenCv manager
            if(hardwareMap != null)
            {
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,
                        hardwareMap.appContext,
                        mLoaderCallback);
            }
        } else
        {
            RobotLog.ii("SJH", "OpenCVLoader loaded from app pkg");
            setup();
        }

        initializing = false;
        initialized  = true;
    }

    public void cleanupCamera()
    {
        if (openCVCamera != null)
        {
            RobotLog.dd("SJH", "cleanupCamera");
            openCVCamera.disableView();
            openCVCamera.disconnectCamera();
        }
    }

    public void setFlipImage(boolean flipImage) {this.flipImage = flipImage;}

    public void setImageProcessor(ImageProcessor imgProc)
    {
        this.imgProc = imgProc;
        RobotLog.dd("SJH_ocvInit", "Image Processor = " + this.imgProc.getName());
    }

    public ImageProcessor getImageProcessor() { return imgProc; }

    public boolean isNewImageReady()
    {
        boolean newImageReady = newImage;
        newImage = false;
        return newImageReady;
    }

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
