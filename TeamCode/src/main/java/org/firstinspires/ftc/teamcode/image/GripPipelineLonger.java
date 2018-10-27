package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
* GripPipelineLonger class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
@SuppressWarnings("unused")
public class GripPipelineLonger
{
	private static final String TAG = "SJH_GPL";
	//Outputs
	private Mat resizeImageOutput;
	private Mat roiMat;
    private Mat blurOutput;
	private Mat cvErodeOutput = new Mat();
	private Mat cvDilateOutput = new Mat();
	private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
	private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();

	public void sizeSource(Mat source0)
	{
		RobotLog.dd(TAG, "Processing image WXH= %dx%d", source0.cols(), source0.rows());
		roiMat = new Mat(source0, new Rect(0, (int)(0.4 * source0.height()),
				source0.width(), source0.height()/3));
		RobotLog.dd(TAG, " roiMat image WXH= %dx%d", roiMat.cols(), roiMat.rows());

		int resizeImageWidth = 512;
		int resizeImageHeight = (int)(resizeImageWidth*((double)roiMat.height()/roiMat.width()));

		if(resizeImageOutput == null)
			resizeImageOutput = new Mat(resizeImageHeight, resizeImageWidth, source0.type());

		int resizeImageInterpolation = Imgproc.INTER_LINEAR;

		resizeImage(roiMat, resizeImageWidth, resizeImageHeight,
				resizeImageInterpolation, resizeImageOutput);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	public void processGold(Mat goldSource)
	{
		@SuppressWarnings("UnnecessaryLocalVariable")
        Mat blurInput = goldSource;
        BlurType blurType = BlurType.get("Gaussian Blur");
        double blurRadius = 4.0;
        if(blurOutput == null)
            blurOutput = new Mat(blurInput.rows(), blurInput.cols(), blurInput.type());

        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {8, 44};
        double[] hsvThresholdSaturation = {121.0, 255.0};
        double[] hsvThresholdValue = {148.0, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step CV_erode0:
		Mat cvErodeSrc = hsvThresholdOutput;
		Mat cvErodeKernel = new Mat();
		Point cvErodeAnchor = new Point(0, 0);
		double cvErodeIterations = 1.0;
		int cvErodeBordertype = Core.BORDER_CONSTANT;
		Scalar cvErodeBordervalue = new Scalar(-1);
		cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

		// Step CV_dilate0:
		Mat cvDilateSrc = cvErodeOutput;
		Mat cvDilateKernel = new Mat();
		Point cvDilateAnchor = new Point(-1, -1);
		double cvDilateIterations = 1.0;
		int cvDilateBordertype = Core.BORDER_CONSTANT;
		Scalar cvDilateBordervalue = new Scalar(-1);
		cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

		// Step Find_Contours0:
        Mat findContoursInput = cvDilateOutput;
		boolean findContoursExternalOnly = false;
		//noinspection ConstantConditions
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Convex_Hulls0:
		ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
		convexHulls(convexHullsContours, convexHullsOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = convexHullsOutput;
		double filterContoursMinArea = 500.0;
		double filterContoursMinPerimeter = 0.0;
		double filterContoursMinWidth = 20.0;
		double filterContoursMaxWidth = 100.0;
		double filterContoursMinHeight = 20.0;
		double filterContoursMaxHeight = 100.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 200.0;
		double filterContoursMinVertices = 0.0;
		double filterContoursMinRatio = 0.5;
		double filterContoursMaxRatio = 1.7;
		filterContours(filterContoursContours,
                filterContoursMinArea,
                filterContoursMinPerimeter,
                filterContoursMinWidth, filterContoursMaxWidth,
                filterContoursMinHeight, filterContoursMaxHeight,
                filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices,
                filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
	}

	public void processSilver(Mat goldSource)
	{
		@SuppressWarnings("UnnecessaryLocalVariable")
		Mat blurInput = goldSource;
		BlurType blurType = BlurType.get("Gaussian Blur");
		double blurRadius = 4.0;
		if(blurOutput == null)
			blurOutput = new Mat(blurInput.rows(), blurInput.cols(), blurInput.type());

		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step HSV_Threshold0:
		Mat hsvThresholdInput = blurOutput;
		double[] hsvThresholdHue = {0.0, 55.0};
		double[] hsvThresholdSaturation = {0.0, 78.0};
		double[] hsvThresholdValue = {168.0, 255.0};
		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step CV_erode0:
		Mat cvErodeSrc = hsvThresholdOutput;
		Mat cvErodeKernel = new Mat();
		Point cvErodeAnchor = new Point(0, 0);
		double cvErodeIterations = 1.0;
		int cvErodeBordertype = Core.BORDER_CONSTANT;
		Scalar cvErodeBordervalue = new Scalar(-1);
		cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

		// Step CV_dilate0:
		Mat cvDilateSrc = cvErodeOutput;
		Mat cvDilateKernel = new Mat();
		Point cvDilateAnchor = new Point(-1, -1);
		double cvDilateIterations = 1.0;
		int cvDilateBordertype = Core.BORDER_CONSTANT;
		Scalar cvDilateBordervalue = new Scalar(-1);
		cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

		// Step Find_Contours0:
		Mat findContoursInput = cvDilateOutput;
		boolean findContoursExternalOnly = false;
		//noinspection ConstantConditions
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		// Step Convex_Hulls0:
		ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
		convexHulls(convexHullsContours, convexHullsOutput);

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = convexHullsOutput;
		double filterContoursMinArea = 500.0;
		double filterContoursMinPerimeter = 0.0;
		double filterContoursMinWidth = 20.0;
		double filterContoursMaxWidth = 100.0;
		double filterContoursMinHeight = 20.0;
		double filterContoursMaxHeight = 100.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 200.0;
		double filterContoursMinVertices = 0.0;
		double filterContoursMinRatio = 0.5;
		double filterContoursMaxRatio = 1.7;
		filterContours(filterContoursContours,
				filterContoursMinArea,
				filterContoursMinPerimeter,
				filterContoursMinWidth, filterContoursMaxWidth,
				filterContoursMinHeight, filterContoursMaxHeight,
				filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices,
				filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
	}
    public void processPit(Mat pitSource)
	{
        @SuppressWarnings("UnnecessaryLocalVariable")
		Mat blurInput = pitSource;
        BlurType blurType = BlurType.get("Gaussian Blur");
        double blurRadius = 4.0;
        if(blurOutput == null)
            blurOutput = new Mat(blurInput.rows(), blurInput.cols(), blurInput.type());

        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {12, 48};
        double[] hsvThresholdSaturation = {0.0, 44.0};
        double[] hsvThresholdValue = {34.0, 105.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step CV_erode0:
        Mat cvErodeSrc = hsvThresholdOutput;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(0, 0);
        double cvErodeIterations = 1.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step CV_dilate0:
        Mat cvDilateSrc = cvErodeOutput;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 1.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

        // Step Find_Contours0:
        Mat findContoursInput = cvDilateOutput;
        boolean findContoursExternalOnly = false;
        //noinspection ConstantConditions
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

        // Step Convex_Hulls0:
        ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
        convexHulls(convexHullsContours, convexHullsOutput);

        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours = convexHullsOutput;
        double filterContoursMinArea = 1000.0;
        double filterContoursMinPerimeter = 0.0;
        double filterContoursMinWidth = 75.0;
        double filterContoursMaxWidth = 512.0;
        double filterContoursMinHeight = 20.0;
        double filterContoursMaxHeight = 200.0;
        double[] filterContoursSolidity = {0, 100};
        double filterContoursMaxVertices = 200.0;
        double filterContoursMinVertices = 0.0;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 100;
        filterContours(filterContoursContours,
                filterContoursMinArea,
                filterContoursMinPerimeter,
                filterContoursMinWidth, filterContoursMaxWidth,
                filterContoursMinHeight, filterContoursMaxHeight,
                filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices,
                filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
    }

    public Rect findMask(ArrayList<MatOfPoint> contours)
    {
        Rect mask = new Rect(0,0,512,1);
        Iterator<MatOfPoint> each = contours.iterator();
        Rect bounded_box;
        int sumtop = 0;
        if(contours.size() == 0) return mask;
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            bounded_box = Imgproc.boundingRect(wrapper);
            sumtop += bounded_box.y;
        }
        mask.height = sumtop / contours.size();
        return mask;
    }

    public Rect leftMask()
	{
        double xMaskPct = 0.35;
	    int height = 200;
	    int width  = 512;

	    if(resizeImageOutput != null)
        {
            height = resizeImageOutput.height();
            width  = (int)(resizeImageOutput.width() * xMaskPct);
        }
		return new Rect(0, 0, width, height);
	}

    public Rect rightMask()
    {
        double xMaskPct = 0.35;
        int height = 200;
        int width  = 512;
        int x = width - (int)(xMaskPct * width);

        if(resizeImageOutput != null)
        {
            height = resizeImageOutput.height();
            width  = (int)(resizeImageOutput.width() * xMaskPct);
        }
        return new Rect(x, 0, width, height);
    }

	public Mat roiMat()                                 { return roiMat; }
	public Mat resizeImageOutput()                      { return resizeImageOutput; }
    public Mat blurOutput()                             { return blurOutput; }
	public Mat cvErodeOutput()                          { return cvErodeOutput; }
	public Mat cvDilateOutput()                         { return cvDilateOutput; }
	public Mat hsvThresholdOutput()                     { return hsvThresholdOutput; }
	public ArrayList<MatOfPoint> findContoursOutput()   { return findContoursOutput; }
	public ArrayList<MatOfPoint> convexHullsOutput()    { return convexHullsOutput; }
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}


	private void resizeImage(Mat input, double width, double height,
							 int interpolation, Mat output)
	{
		RobotLog.dd(TAG, " resizeImage image inWXH= %dx%d outWXH= %dx%d ", input.cols(),
				input.rows(), output.cols(), output.rows());
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}

	enum BlurType
    {
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
		BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type)
        {
			if (BILATERAL.label.equals(type))     { return BILATERAL; }
			else if (GAUSSIAN.label.equals(type)) { return GAUSSIAN;  }
			else if (MEDIAN.label.equals(type))   { return MEDIAN;    }
			else                                  { return BOX;       }
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	private void blur(Mat input, BlurType type, double doubleRadius,
					  Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;

		RobotLog.dd(TAG, " blur image inWXH= %dx%d outWXH= %dx%d", input.cols(),
				input.rows(), output.cols(), output.rows());

		switch(type)
        {
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
		int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null) {
			borderValue = new Scalar(-1);
		}
		Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
	int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null){
			borderValue = new Scalar(-1);
		}
		Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param out The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Compute the convex hulls of contours.
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	private void convexHulls(List<MatOfPoint> inputContours,
		ArrayList<MatOfPoint> outputContours) {
		final MatOfInt hull = new MatOfInt();
		outputContours.clear();
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final MatOfPoint mopHull = new MatOfPoint();
			Imgproc.convexHull(contour, hull);
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			outputContours.add(mopHull);
		}
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}




}

