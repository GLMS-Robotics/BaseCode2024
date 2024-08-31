package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
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

    // Saved from init for bitmap rendering in preview
    private int w;
    private int h;

    // Individual channel thresholds for preview
    public Mat th = new Mat();
    public Mat ts = new Mat();
    public Mat tv = new Mat();

    /**
     * Set up the pipeline.
     * @param width Width of incoming images
     * @param height Height of incoming images
     * @param calibration Camera calibration data
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        w = width;
        h = height;
    }

    /**
     * Process a single picture from the camera.
     * @param frame The incoming picture
     * @param captureTimeNanos Time in nanoseconds it took to take the picture
     * @return Data to be passed to onDrawFrame
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // Blur the image slightly
        Imgproc.GaussianBlur(frame, blurred, new Size(11,11), 1.0);


        // To HSV
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);


        // Threshold the image
        // Note: This one is in Core, not Imgproc
        Core.inRange(hsv, minColor(), maxColor(), thresholded);


        contours.clear();
        // Find contours (blobs) in the image
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


        /*
        Now we need to pick the biggest "contour" on the image.
        A "contour" is just a fancy word for a blob of color, or the outlines around it.
        Using a loop, go through the `contours` variable and set `biggestIndex` to the
        index of the largest one.
        Also, make sure to ignore contours less than `minArea()` in size!
         */
        // Pick the biggest contour
        int biggestIndex = -1;
        double biggestArea = 0.0;

        for(int j=0; j < contours.size(); j++)
        {
            double area = Imgproc.contourArea(contours.get(j));
            // Is this the biggest one yet?
            if(area > biggestArea && area > minArea()) {
                // Yes, keep this contour
                biggestIndex = j;
                biggestArea = area;
            }
        }

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


        // By converting to an array, this creates a new object that won't
        // cause multithreading insanity when accessed by onDrawFrame
        return contours.toArray();
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

        // Prevent zombie mode if pipeline is switched off between process and draw
        if(userContext != null) {
            // Set up paint
            Paint p = new Paint();
            p.setColor(Color.BLUE);
            p.setStrokeWidth(5);
            p.setStyle(Paint.Style.STROKE);

            // Draw bounding box
            if (hasResult) {
                canvas.drawRect(boundingBox.x * scaleBmpPxToCanvasPx, boundingBox.y * scaleBmpPxToCanvasPx,
                        (boundingBox.width + boundingBox.x) * scaleBmpPxToCanvasPx, (boundingBox.height + boundingBox.y) * scaleBmpPxToCanvasPx, p);
            }

            // Manually draw contour points
            p.setColor(Color.YELLOW);
            p.setStrokeWidth(3);
            for (Object o : (Object[]) userContext) {
                MatOfPoint contour = (MatOfPoint) o;
                Point[] pts = contour.toArray();

                if (pts.length > 1) {
                    for (int j = 0; j < pts.length - 1; j++) {
                        //canvas.drawPoint((float) pts[j].x * scaleBmpPxToCanvasPx, (float) pts[j].y * scaleBmpPxToCanvasPx, p);
                        canvas.drawLine((float) pts[j].x * scaleBmpPxToCanvasPx, (float) pts[j].y * scaleBmpPxToCanvasPx,
                                (float) pts[j + 1].x * scaleBmpPxToCanvasPx, (float) pts[j + 1].y * scaleBmpPxToCanvasPx,
                                p);
                    }
                    canvas.drawLine((float) pts[0].x * scaleBmpPxToCanvasPx, (float) pts[0].y * scaleBmpPxToCanvasPx,
                            (float) pts[pts.length - 1].x * scaleBmpPxToCanvasPx, (float) pts[pts.length - 1].y * scaleBmpPxToCanvasPx,
                            p);
                }
            }

            // Draw threshold in corner
            Bitmap camOut = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            Utils.matToBitmap(thresholded, camOut);
            canvas.drawBitmap(camOut, null, new android.graphics.Rect((int) (onscreenWidth * 0.75),
                    (int) (onscreenHeight * 0.75),
                    (int) (onscreenWidth),
                    (int) (onscreenHeight)), null);

            // Draw HSV in corner
            camOut = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            Utils.matToBitmap(hsv, camOut);
            canvas.drawBitmap(camOut, null, new android.graphics.Rect((int) (onscreenWidth * 0.75),
                    (int) (onscreenHeight * 0.5),
                    (int) (onscreenWidth),
                    (int) (onscreenHeight * 0.75)), null);


            // Draw individual H,S, and V thresholds

            Core.inRange(hsv, new Scalar(minColor().val[0], 0, 0), new Scalar(maxColor().val[0], 255, 255), th);
            Core.inRange(hsv, new Scalar(0, minColor().val[1], 0), new Scalar(255, maxColor().val[1], 255), ts);
            Core.inRange(hsv, new Scalar(0, 0, minColor().val[2]), new Scalar(255, 255, maxColor().val[2]), tv);

            camOut = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            Utils.matToBitmap(th, camOut);
            canvas.drawBitmap(camOut, null, new android.graphics.Rect((int) (0),
                    (int) (0),
                    (int) (onscreenWidth * 0.20),
                    (int) (onscreenHeight * 0.15)), null);

            camOut = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            Utils.matToBitmap(ts, camOut);
            canvas.drawBitmap(camOut, null, new android.graphics.Rect((int) (onscreenWidth * 0.20),
                    (int) (0),
                    (int) (onscreenWidth * 0.40),
                    (int) (onscreenHeight * 0.15)), null);

            camOut = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            Utils.matToBitmap(tv, camOut);
            canvas.drawBitmap(camOut, null, new android.graphics.Rect((int) (onscreenWidth * 0.40),
                    (int) (0),
                    (int) (onscreenWidth * 0.60),
                    (int) (onscreenHeight * 0.15)), null);


            // Draw color blocks for min and max
            p.setStyle(Paint.Style.FILL);
            p.setColor(Color.HSVToColor(new float[]{(float) (minColor().val[0] * 360.0 / 255.0), (float) (minColor().val[1] / 255.0), (float) (minColor().val[2] / 255.0)}));
            canvas.drawRect(new android.graphics.Rect((int) (onscreenWidth * 0.45),
                    (int) (onscreenHeight * 0.95),
                    (int) (onscreenWidth * 0.5),
                    (int) (onscreenHeight)), p);

            p.setColor(Color.HSVToColor(new float[]{(float) (maxColor().val[0] * 360.0 / 255.0), (float) (maxColor().val[1] / 255.0), (float) (maxColor().val[2] / 255)}));
            canvas.drawRect(new android.graphics.Rect((int) (onscreenWidth * 0.5),
                    (int) (onscreenHeight * 0.95),
                    (int) (onscreenWidth * 0.55),
                    (int) (onscreenHeight)), p);

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
