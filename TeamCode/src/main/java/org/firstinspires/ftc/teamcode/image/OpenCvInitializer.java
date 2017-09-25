package org.firstinspires.ftc.teamcode.image;


import android.app.Activity;
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
    private CommonUtil com = CommonUtil.getInstance();
    private boolean newImage = false;
    private ImageProcessor imgProc = null;
    private static JavaCameraView openCVCamera = null;

    private Activity act = null;

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

    public OpenCvInitializer(boolean addJavaCameraView)
    {
        this.addJavaCameraView = addJavaCameraView;

        act = com.getActivity();

        initOpenCv();
    }

    public void setupCameraView()
    {
        if(!loaded || !addJavaCameraView || openCVCamera != null)
        {
            RobotLog.dd("SJH", "OpenCvLoad: JCV not requested or already setup");
            return;
        }

        RobotLog.dd("SJH", "OpenCvLoad: setting up JCV");
        class CameraRunnable implements Runnable
        {
            private JavaCameraView jcv = null;
            private JavaCameraView getJcv() { return jcv; }

            public void run() {
                RobotLog.dd("SJH", "Setting up JavaCameraView");

                //Add a JavaCameraView child to cameraMonitorView
                ViewGroup vg = act.findViewById(com.getCameraMonitorViewId());

                jcv = new JavaCameraView(act, CameraBridgeViewBase.CAMERA_ID_BACK);
                jcv.setMaxFrameSize(480, 320);

                ViewGroup.LayoutParams vglop =
                        new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                                                   ViewGroup.LayoutParams.MATCH_PARENT);
                jcv.setLayoutParams(vglop);
                vg.addView(jcv, 0);
            }
        }

        initializing = true;
        initialized = false;
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

        initializing = false;
        initialized  = true;
    }

    private void setup()
    {
        setupCameraView();
    }

    private void initOpenCv()
    {
        if(loaded || loading)
        {
            RobotLog.dd("SJH", "OpenCV already loaded");
            //setup();
            return;
        }

        loading = true;

        BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(act)
        {
            @Override
            public void onManagerConnected(int status)
            {
                switch (status)
                {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        RobotLog.ii("SJH", "OpenCVLoad: SUCCESS");
                        initialized = true;
                        initialized = false;
                        //setup();
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

        //Try loading ocv libraries from within app pkg
        if (!OpenCVLoader.initDebug())
        {
            RobotLog.dd("SJH", "OpenCvLoad: Could not load from app - trying OpenCV manager");
            //Try loading using OpenCv manager
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0,
                    act, mLoaderCallback);
        } else
        {
            RobotLog.ii("SJH", "OpenCVLoader loaded from app pkg");
            //setup();
        }
    }

    public void cleanupCamera()
    {
        com.restoreLayout();
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
