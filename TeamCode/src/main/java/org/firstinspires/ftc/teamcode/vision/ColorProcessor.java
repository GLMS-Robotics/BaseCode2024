package org.firstinspires.ftc.teamcode.vision;

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

import java.util.ArrayList;
import java.util.List;

public abstract class ColorProcessor implements VisionProcessor {

    // Does the vision processor see anything?
    private boolean hasResult = false;

    // Intermediate images saved by the pipeline
    // They are instance variable up here to avoid creating and throwing away
    // new pictures every time
    public Mat blurred = new Mat();
    public Mat hsv = new Mat();
    public Mat thresholded = new Mat();

    // Contours detected in the thresholded mat
    private List<MatOfPoint> contours = new ArrayList<>();

    // Bounding box around the biggest detected object
    private Rect boundingBox = new Rect();

    // Center of the bounding box
    private Point center = new Point();


    /**
     * Set up the pipeline.
     * @param width Width of incoming images
     * @param height Height of incoming images
     * @param calibration Camera calibration data
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // We don't need anything here right now
    }

    /**
     * Process a single picture from the camera.
     * @param frame The incoming picture
     * @param captureTimeNanos Time in nanoseconds it took to take the picture
     * @return Data to be passed to onDrawFrame
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // TODO Fill in the image processing steps from the Imgproc libarary

        // Blur the image slightly
        // Params: (frame, blurred, new Size(11,11), 1.0)


        // To HSV
        // Params: (blurred, hsv, Imgproc.COLOR_RGB2HSV)


        // Threshold the image
        // Note: This one is in Core, not Imgproc
        // Params: (hsv, minColor(), maxColor(), thresholded)


        contours.clear();
        // Find contours (blobs) in the image
        // Params: (thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)


        /*
        TODO
        Now we need to pick the biggest "contour" on the image.
        A "contour" is just a fancy word for a blob of color, or the outlines around it.
        Using a loop, go through the `contours` variable and set `biggestIndex` to the
        index of the largest one.
        Also, make sure to ignore contours less than `minArea()` in size!
        HINT 1: You will need a loop!
        HINT 2: contours.get(7) gets contour index 7
        HINT 3: Imgproc.contourArea(c) calculates the area of contour c
         */
        // Pick the biggest contour
        int biggestIndex = -1;
        double biggestArea = 0.0;
        //???



        // Did we find anything?
        if(biggestIndex >= 0)
        {
            hasResult = true;

            boundingBox = Imgproc.boundingRect(contours.get(biggestIndex));
            center = new Point(
                    (boundingBox.br().x + boundingBox.tl().x)/2,
                    (boundingBox.br().y + boundingBox.tl().y)/2);
        }
        else
        {
            hasResult = false;
        }


        return contours;
    }

    /**
     * Draw the results of the vision processor on the robot's "screen".
     * @param canvas Canvas for drawing on
     * @param onscreenWidth Actual screen width
     * @param onscreenHeight Actual screen height
     * @param scaleBmpPxToCanvasPx Actual screen size / input size
     * @param scaleCanvasDensity Undocumented
     * @param userContext Object passed from processedFrame
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Set up paint
        Paint p = new Paint();
        p.setColor(Color.BLUE);
        p.setStrokeWidth(5);
        p.setStyle(Paint.Style.STROKE);

        // Draw bounding box
        if(hasResult) {
            canvas.drawRect(boundingBox.x * scaleBmpPxToCanvasPx, boundingBox.y * scaleBmpPxToCanvasPx,
                    (boundingBox.width + boundingBox.x) * scaleBmpPxToCanvasPx, (boundingBox.height + boundingBox.y) * scaleBmpPxToCanvasPx, p);
        }

        // Manually draw contour points
        // This should be upgraded to draw lines at some point
        p.setColor(Color.YELLOW);
        for (MatOfPoint contour : contours) {
            Point[] pts = contour.toArray();

            for(int j=0; j<pts.length;j++)
            {
                canvas.drawPoint((float) pts[j].x * scaleBmpPxToCanvasPx,(float) pts[j].y * scaleBmpPxToCanvasPx, p);
            }
        }
    }

    /**
     * Does this color processor see anything?
     * @return True if this processor sees something, false otherwise
     */
    public boolean hasResult()
    {
        return hasResult;
    }

    /**
     * Get the bounding box surrounding the biggest thing we see.
     * @return a bounding box around the biggest object seen
     */
    public Rect getBoundingBox()
    {
        return boundingBox;
    }


    /**
     * Get a point representing the center of the largest object seen.
     * This is the center of the bounding box, not the center of mass.
     * @return A point representing the center of the object.
     */
    public Point getCenter()
    {
        return center;
    }




    /**
     * Get a scalar containing the maximum Hue, Saturation, and Value of pixels that match
     * our target color.
     * @return a Scalar containing the maximum desired Hue, Saturation, and Value.
     */
    public abstract Scalar maxColor();

    /**
     * Get a scalar containing the minimum Hue, Saturation, and Value of pixels that match
     * our target color.
     * @return a Scalar containing the minimum desired Hue, Saturation, and Value.
     */
    public abstract Scalar minColor();

    /**
     * Get the minimum area that the object we are looking for should take up on the camera.
     * @return The least area our target should take up. Anything smaller than this will be ignored.
     */
    public abstract double minArea();


}
