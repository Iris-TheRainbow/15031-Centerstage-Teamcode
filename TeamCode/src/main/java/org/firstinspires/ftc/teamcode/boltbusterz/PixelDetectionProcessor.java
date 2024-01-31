package org.firstinspires.ftc.teamcode.boltbusterz;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class PixelDetectionProcessor implements VisionProcessor {
	private final ArrayList<MatOfPoint> contours = new ArrayList<>();
	Mat yellowMat = new Mat();
	Mat purpleMat = new Mat();
	Mat greenMat = new Mat();
	Mat whiteMat = new Mat();
	Mat thresh = new Mat();
	Mat hierarchy = new Mat();
	public double largestContourY, largestContourX;
	public float targetX = 110;
	public int kernelSize = 2, kernelSize2 = 7;
	private Paint greenPaint, redPaint, greenPaintThin;
	private double largestContourArea;
	private MatOfPoint largestContour;
	public PixelDetectionProcessor(float target) {
		greenPaint = new Paint();
		greenPaint.setColor(Color.GREEN); // you may want to change this
		greenPaint.setAntiAlias(true);
		greenPaint.setStrokeWidth(10); // or this
		greenPaint.setStrokeCap(Paint.Cap.ROUND);
		greenPaint.setStrokeJoin(Paint.Join.ROUND);
		redPaint = new Paint();
		redPaint.setColor(Color.RED); // you may want to change this
		redPaint.setAntiAlias(true);
		redPaint.setStrokeWidth(5); // or this
		greenPaintThin = new Paint();
		greenPaintThin.setColor(Color.GREEN); // you may want to change this
		greenPaintThin.setAntiAlias(true);
		greenPaintThin.setStrokeWidth(5); // or this
		targetX = target;
	}

	@Override
	public void init(int width, int height, CameraCalibration calibration) {

	}

	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

		Scalar yellowLower = new Scalar(20, 100, 200);
		Scalar yellowUpper = new Scalar(30, 255, 255);
		Core.inRange(frame, yellowLower, yellowUpper, yellowMat);

		Scalar greenLower = new Scalar(30, 60, 90);
		Scalar greenUpper = new Scalar(110, 160, 255);
		Core.inRange(frame, greenLower, greenUpper, greenMat);

		Scalar purpleLower = new Scalar(100, 12, 230);
		Scalar purpleUpper = new Scalar(165, 80, 255);
		Core.inRange(frame, purpleLower, purpleUpper, purpleMat);

		Scalar whiteLower = new Scalar(0, 0, 200);
		Scalar whiteUpper = new Scalar(80, 12, 255);
		Core.inRange(frame, whiteLower, whiteUpper, whiteMat);

		Mat tempMat1 = new Mat();
		Mat tempMat2 = new Mat();
		Core.bitwise_or(yellowMat, greenMat, tempMat1);
		yellowMat.release();
		greenMat.release();

		Core.bitwise_or(purpleMat, whiteMat, tempMat2);
		purpleMat.release();
		whiteMat.release();

		Core.bitwise_or(tempMat1, tempMat2, thresh);
		tempMat1.release();
		tempMat2.release();
		Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, new Size(2 * kernelSize + 1, 2 * kernelSize + 1),
				new Point(kernelSize, kernelSize));
		Mat element2 = Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, new Size(2 * kernelSize2 + 1, 2 * kernelSize2 + 1),
				new Point(kernelSize2, kernelSize2));
		Imgproc.dilate(thresh, frame, element2);
		Imgproc.erode(frame, frame, element);
		Imgproc.dilate(frame, frame, element2);
		contours.clear();

		// this finds the contours, which are borders between black and white, and tries to simplify them to make nice outlines around potential objects
		// this basically helps us to find all the shapes/outlines of objects that exist within our colour range
		Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

		// this sets up our largest contour area to be 0
		largestContourArea = -1;
		// and our currently found largest contour to be null
		largestContour = null;

		// gets the current minimum area from min area
		double minArea = 10;

		// finds the largest contour!
		// for each contour we found before we loop over them, calculate their area,
		// and then if our area is larger than our minimum area, and our currently found largest area
		// it stores the contour as our largest contour and the area as our largest area
		for (MatOfPoint contour : contours) {
			double area = Imgproc.contourArea(contour);
			if (area > largestContourArea && area > minArea) {
				largestContour = contour;
				largestContourArea = area;
			}
		}


		// sets up the center points of our largest contour to be -1 (offscreen)
		largestContourX = largestContourY = -1;

		// if we found it, calculates the actual centers
		if (largestContour != null) {
			Moments moment = Imgproc.moments(largestContour);
			largestContourX = (moment.m10 / moment.m00);
			largestContourY = (moment.m01 / moment.m00);
		}
		return frame;
	}

	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
		// this method draws the rectangle around the largest contour and puts the current prop position into that rectangle
		// you don't need to call it

//		for (MatOfPoint contour : contours) {
//			Rect rect = Imgproc.boundingRect(contour);
//			canvas.drawLines(new float[]{rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx}, textPaint);
//		}

		// if the contour exists, draw a rectangle around it and put its position in the middle of the rectangle
		if (largestContour != null) {
			Rect rect = Imgproc.boundingRect(largestContour);

			float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};

			canvas.drawLine(points[0], points[1], points[0], points[3], greenPaint);
			canvas.drawLine(points[0], points[1], points[2], points[1], greenPaint);

			canvas.drawLine(points[0], points[3], points[2], points[3], greenPaint);
			canvas.drawLine(points[2], points[1], points[2], points[3], greenPaint);
			if (largestContourX <= targetX + 10 && largestContourX >= targetX - 10) {
				canvas.drawLine((targetX*scaleBmpPxToCanvasPx), 0, (targetX * scaleBmpPxToCanvasPx), 1000, greenPaintThin);
			}
			else {
				canvas.drawLine((targetX * scaleBmpPxToCanvasPx), 0, (targetX * scaleBmpPxToCanvasPx), 1000, redPaint);
			}
		}
	}
	public double getPixelX(){
		return largestContourX;
	}
	public void close(){ hierarchy.release(); }
}
