package org.firstinspires.ftc.teamcode.image;


import android.app.Activity;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.CommonUtil;
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
    private ImageProcessor imgProc = null;
    private static CameraBridgeViewBase openCVCamera = null;

    private boolean flipImage           = false;
    private boolean addJavaCameraView   = true;
    private static boolean loaded       = false;
    private static boolean loading      = false;
    private static boolean initialized  = false;
    private static boolean initializing = false;

    private int width;
    private int height;

    int getWidth()  {return  width;}
    int getHeight() {return height;}
    public  static boolean isInitializing() {return initializing;}
    private static boolean isInitialized()  {return initialized;}

    private final static String TAG = "SJH_OCVI";

    static
    {
        RobotLog.dd(TAG, "Calling initOpenCv");
        initOpenCv();
    }

    public OpenCvInitializer(boolean addJavaCameraView)
    {
        this.addJavaCameraView = addJavaCameraView;
    }

    public void setupCameraView()
    {
        if(openCVCamera != null)
        {
            RobotLog.dd(TAG, "OCVI_setupCameraView: JCV already setup - just update");
            CommonUtil.getInstance().setVisibility(openCVCamera, View.VISIBLE);
            if(addJavaCameraView)
            {
                openCVCamera.enableView();
                RobotLog.dd(TAG, "Set cameraViewListener");
                openCVCamera.setCvCameraViewListener(this);
            }
            return;
        }

        RobotLog.dd(TAG, "OpenCvLoad: setting up JCV");
        class CameraRunnable implements Runnable
        {
            private CameraBridgeViewBase jcv = null;
            private CameraBridgeViewBase getJcv() { return jcv; }
            private CommonUtil com = CommonUtil.getInstance();
            private Activity act = com.getActivity();
            private int camMonViewId = com.getCameraMonitorViewId();

            public void run() {
                RobotLog.dd(TAG, "Setting up JavaCameraView in thread");

                //Add a JavaCameraView child to cameraMonitorView
                ViewGroup vg = act.findViewById(camMonViewId);

                RobotLog.dd(TAG, "Creating JavaCameraView");
                jcv = new JavaCameraView(act, CameraBridgeViewBase.CAMERA_ID_BACK);
                RobotLog.dd(TAG, "setMaxFrameSize");
                jcv.setMaxFrameSize(480, 320);


                //ViewGroup.LayoutParams.MATCH_PARENT
                ViewGroup.LayoutParams vglop =
                        new ViewGroup.LayoutParams(480,
                                                   320);

                RobotLog.dd(TAG, "setLayoutParams");
                jcv.setLayoutParams(vglop);
                RobotLog.dd(TAG, "Adding jcv to cameraMonitorView");
                vg.addView(jcv, 0);
            }
        }

        initializing = true;
        initialized = false;
        RobotLog.dd(TAG, "Creating runnable instance");
        CameraRunnable mr = new CameraRunnable();
        RobotLog.dd(TAG, "Running on UI thread");

        CommonUtil.getInstance().getActivity().runOnUiThread(mr);
        RobotLog.dd(TAG, "Started thread");

        CameraBridgeViewBase jcv = null;
        ElapsedTime jtimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(jcv == null && jtimer.milliseconds() < 5000)
        {
            jcv = mr.getJcv();
        }

        if(jcv == null)
        {
            RobotLog.ww(TAG, "JavaCameraView not created");
            return;
        }

        RobotLog.dd(TAG, "Got JavaCameraView");

        openCVCamera = jcv;
        RobotLog.dd(TAG, "Set visible JavaCameraView");
        openCVCamera.setVisibility(CameraBridgeViewBase.VISIBLE);
        RobotLog.dd(TAG, "Set cameraViewListener");
        openCVCamera.setCvCameraViewListener(this);

        openCVCamera.enableFpsMeter();

        width = openCVCamera.getMeasuredWidth();
        height = openCVCamera.getMeasuredHeight();
        RobotLog.ii(TAG, "CAMERA %d x %d", width, height);

        RobotLog.dd(TAG, "OpenCvLoad: enabling view");
        openCVCamera.enableView();
        RobotLog.dd(TAG, "OpenCvLoad: called enableView");

        initializing = false;
        initialized  = true;
    }

    private static void initOpenCv()
    {
        //initOpenCv loads OpenCV libraries needed for image analysis
        //This should only need to be done once per
        //RobotController run
        CommonUtil loccom = CommonUtil.getInstance();
        if(loaded)
        {
            RobotLog.dd(TAG, "OpenCV already loaded");
            return;
        }

        if(loading)
        {
            ElapsedTime loadTimer = new ElapsedTime();
            while(loading && loadTimer.seconds() < 2)
            {
                RobotLog.dd(TAG, "OpenCV still loading");

                if(loccom.getLinearOpMode() != null)
                {
                    loccom.getLinearOpMode().sleep(2);
                }
                else
                {
                    try
                    {
                        Thread.sleep(2);
                    } catch (InterruptedException e)
                    {
                        e.printStackTrace();
                    }
                }
            }
        }

        RobotLog.dd(TAG, "Creating LoaderCallback");
        loading = true;

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(loccom.getActivity())
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status)
                {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        RobotLog.ii(TAG, "OpenCVLoad: SUCCESS");
                        loaded = true;
                        loading = false;
                    }
                    break;
                    default:
                    {
                        RobotLog.dd(TAG, "OpenCvLoad: FAILURE");
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        RobotLog.dd(TAG, "Loading OpenCV - static first");
        //Try loading ocv libraries from within app pkg
        if (!OpenCVLoader.initDebug())
        {
            RobotLog.dd(TAG, "OpenCvLoad: Could not load from app - trying OpenCV manager");
            //Try loading using OpenCv manager
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,
                    loccom.getActivity(), mLoaderCallback);
        } else
        {
            RobotLog.ii(TAG, "OpenCVLoader loaded from app pkg");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        RobotLog.dd(TAG, "OpenCv loaded");
    }

    public void cleanupCamera()
    {
        CommonUtil com = CommonUtil.getInstance();
        com.restoreLayout();
        if (openCVCamera != null)
        {
            RobotLog.dd(TAG, "cleanupCamera");
            RobotLog.dd(TAG, "Disable view");
            openCVCamera.disableView();
            //RobotLog.dd(TAG, "Disconnect camera");
            //openCVCamera.disconnectCamera();
            com.setVisibility(openCVCamera, View.GONE);
            //com.removeView(openCVCamera);
            initialized = false;
        }
    }

    public void setFlipImage(boolean flipImage) {this.flipImage = flipImage;}

    /*package-private*/ void setImageProcessor(ImageProcessor imgProc)
    {
        this.imgProc = imgProc;
        RobotLog.dd(TAG, "Image Processor = " + this.imgProc.getName());
    }

    public ImageProcessor getImageProcessor() { return imgProc; }

    @Override
    public void onCameraViewStarted(int width, int height)
    {
        RobotLog.ii(TAG, "CAMERA VIEW STARTED %4dx%4d", width, height);
        if (openCVCamera != null)
        {
            RobotLog.dd(TAG, "ocv started - enable view");
            //openCVCamera.enableView();
            CommonUtil.getInstance().setVisibility(openCVCamera, View.VISIBLE);
        }
    }

    @Override
    public void onCameraViewStopped()
    {
        RobotLog.ii(TAG, "CAMERA VIEW STOPPED");
        if(imgProc != null)
        {
            RobotLog.dd(TAG, "ocv stopped - stop Sensing");
            imgProc.stopSensing();
        }

        if (openCVCamera != null)
        {
            RobotLog.dd(TAG, "ocv stopped - disable view");
            openCVCamera.disableView();
            CommonUtil.getInstance().setVisibility(openCVCamera, View.GONE);
        }
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame)
    {
        RobotLog.dd(TAG, "onCameraFrame");

        Mat rgb = inputFrame.rgba();

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
