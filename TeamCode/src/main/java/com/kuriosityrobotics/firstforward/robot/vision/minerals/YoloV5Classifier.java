package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import static org.apache.commons.collections4.ComparatorUtils.reversedComparator;

import static java.util.Comparator.comparing;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.RectF;

import org.apache.commons.collections4.ComparatorUtils;
import org.apache.commons.collections4.queue.CircularFifoQueue;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.gpu.GpuDelegate;
import org.tensorflow.lite.nnapi.NnApiDelegate;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;

public class YoloV5Classifier implements Classifier {
    private static final boolean USE_NON_MAXIUM_SUPPRESSION = false;
    private static final boolean USE_QUANTIZED_MODEL = true;
    public static final int CLOSENESS_THRESHOLD = 20;
    public static final float DETECTION_THRESHOLD = .6f;

    private static final float IMAGE_MEAN = 0;
    private static final float IMAGE_STD = 255.0f;

    // Number of threads in the java app
    private static final int NUM_THREADS = 1;
    // Pre-allocated buffers.
    private static final String[] labels = {"Ball", "Waffle"};
    private static YoloV5Classifier theCargoDetector;
    //config yolo
    private final int inputSize;
    private final int outputRowCount;
    private final boolean isQuantized;
    private final int imageBufferSize, outputBufferSize;
    private final int classCount;
    protected float mNmsThresh = 0.6f;

    private int inputZeroPoint;
    private float inputScale;
    private MappedByteBuffer tfliteModel;
    private Interpreter interpreter;
    private float outputScale;
    private int outputZeroPoint;

    YoloV5Classifier(final AssetManager assetManager,
                     final String modelFilename,
                     final boolean isQuantized,
                     final int inputSize) throws IOException {
        try {
            Interpreter.Options options = (new Interpreter.Options());
            options.setNumThreads(NUM_THREADS);
            this.tfliteModel = Utils.loadModelFile(assetManager, modelFilename);
            this.interpreter = new Interpreter(tfliteModel, options);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        this.isQuantized = isQuantized;
        this.inputSize = inputSize;
        this.outputRowCount = (int) ((Math.pow((inputSize / 32.), 2) + Math.pow((inputSize / 16.), 2) + Math.pow((inputSize / 8.), 2)) * 3);

        int numBytesPerChannel = isQuantized ? 1 : 4;
        this.imageBufferSize = this.inputSize * this.inputSize * 3 * numBytesPerChannel;

        var shape = interpreter.getOutputTensor(0).shape();
        this.classCount = shape[shape.length - 1] - 5;
        this.outputBufferSize = outputRowCount * (classCount + 5) * numBytesPerChannel;

        if (this.isQuantized) {
            var inputTensor = interpreter.getInputTensor(0);
            this.inputScale = inputTensor.quantizationParams().getScale();
            this.inputZeroPoint = inputTensor.quantizationParams().getZeroPoint();

            var outputTensor = interpreter.getOutputTensor(0);
            this.outputScale = outputTensor.quantizationParams().getScale();
            this.outputZeroPoint = outputTensor.quantizationParams().getZeroPoint();
        }

        interpreter.modifyGraphWithDelegate(new GpuDelegate());
    }

    public static YoloV5Classifier getInstance() {
        if (theCargoDetector == null) {
            try {
                theCargoDetector = new YoloV5Classifier(AppUtil.getInstance().getActivity().getAssets(),
                        USE_QUANTIZED_MODEL  ? "best-int8.tflite" : "best-fp16.tflite", USE_QUANTIZED_MODEL, 416);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
        ;

        return theCargoDetector;
    }

    protected void convertBitmapToByteBuffer(Bitmap bitmap, ByteBuffer buffer) {
        var intValues = new int[inputSize * inputSize];

        bitmap.getPixels(intValues, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        buffer.rewind();

        for (int i = 0; i < inputSize; ++i) {
            for (int j = 0; j < inputSize; ++j) {
                int pixelValue = intValues[i * inputSize + j];
                if (isQuantized) {
                    // Quantized model
                    buffer.put((byte) ((((pixelValue >> 16) & 0xFF) - IMAGE_MEAN) / IMAGE_STD / inputScale + inputZeroPoint));
                    buffer.put((byte) ((((pixelValue >> 8) & 0xFF) - IMAGE_MEAN) / IMAGE_STD / inputScale + inputZeroPoint));
                    buffer.put((byte) (((pixelValue & 0xFF) - IMAGE_MEAN) / IMAGE_STD / inputScale + inputZeroPoint));
                } else { // Float model
                    buffer.putFloat((((pixelValue >> 16) & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
                    buffer.putFloat((((pixelValue >> 8) & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
                    buffer.putFloat(((pixelValue & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
                }
            }
        }
    }

    public int getInputSize() {
        return inputSize;
    }

    @Override
    public void enableStatLogging(final boolean logStats) {
    }

    @Override
    public String getStatString() {
        return "";
    }

    @Override
    public void close() {
        interpreter.close();
        interpreter = null;
        tfliteModel = null;
    }

    public void setNumThreads(int num_threads) {
        if (interpreter != null) interpreter.setNumThreads(num_threads);
    }

    @Override
    public void setUseNNAPI(boolean isChecked) {
//        if (tfLite != null) tfLite.setUseNNAPI(isChecked);
    }

    public float getObjThresh() {
        return DETECTION_THRESHOLD;
    }

    //non maximum suppression
    protected ArrayList<Recognition> nms(ArrayList<Recognition> list) {
        ArrayList<Recognition> nmsList = new ArrayList<>();

        for (String label : labels) {
            PriorityQueue<Recognition> pq = new PriorityQueue<>(
                    50,
                    reversedComparator(comparing(Recognition::getConfidence))
            );

            for (int i = 0; i < list.size(); ++i) {
                if (list.get(i).getDetectionType().equals(label)) {
                    pq.add(list.get(i));
                }
            }

            //2.do non maximum suppression
            while (!pq.isEmpty()) {
                var iter = pq.iterator();
                var max = iter.next();
                nmsList.add(max);
                pq.clear();

                while (iter.hasNext()) {
                    Recognition detection = iter.next();
                    RectF b = detection.getLocation();
                    if (intersectionProportion(max.getLocation(), b) < mNmsThresh)
                        pq.add(detection);

                }
            }
        }
        return nmsList;
    }

    protected float intersectionProportion(RectF a, RectF b) {
        return intersection(a, b) / union(a, b);
    }

    protected float intersection(RectF a, RectF b) {
        float w = overlap((a.left + a.right) / 2, a.right - a.left,
                (b.left + b.right) / 2, b.right - b.left);
        float h = overlap((a.top + a.bottom) / 2, a.bottom - a.top,
                (b.top + b.bottom) / 2, b.bottom - b.top);
        if (w < 0 || h < 0) return 0;
        return w * h;
    }

    protected float union(RectF a, RectF b) {
        float i = intersection(a, b);
        return (a.right - a.left) * (a.bottom - a.top) + (b.right - b.left) * (b.bottom - b.top) - i;
    }

    protected float overlap(float x1, float w1, float x2, float w2) {
        float l1 = x1 - w1 / 2;
        float l2 = x2 - w2 / 2;
        float left = Math.max(l1, l2);
        float r1 = x1 + w1 / 2;
        float r2 = x2 + w2 / 2;
        float right = Math.min(r1, r2);
        return right - left;
    }

    public ArrayList<Recognition> recognizeImage(Bitmap bitmap) {
        var imageBuffer = ByteBuffer.allocateDirect(imageBufferSize);
        imageBuffer.order(ByteOrder.nativeOrder());
        convertBitmapToByteBuffer(bitmap, imageBuffer);

        var outputBuffer = ByteBuffer.allocateDirect(this.outputBufferSize);
        outputBuffer.order(ByteOrder.nativeOrder());
        outputBuffer.rewind();

        interpreter.run(imageBuffer, outputBuffer);

        outputBuffer.rewind();

        var detections = new ArrayList<Recognition>();

        // {x, y, w, h, confidence, ball, waffle}
        var dequantizedRows = new float[outputRowCount][classCount + 5];
        for (int i = 0; i < outputRowCount; i++) {
            for (int j = 0; j < classCount + 5; j++)
                if (isQuantized)
                    dequantizedRows[i][j] = outputScale * (((int) outputBuffer.get() & 0xFF) - outputZeroPoint);
                else
                    dequantizedRows[i][j] = outputBuffer.getFloat();

            for (int j = 0; j < 4; j++)
                dequantizedRows[i][j] *= getInputSize();
        }

        for (int i = 0; i < outputRowCount; i++) {
            float confidence = dequantizedRows[i][4];

            var detectedClass = "UNKNOWN";

            var maxClass = -1f;
            for (int c = 0; c < labels.length; c++) {
                if (dequantizedRows[i][5 + c] > maxClass) {
                    detectedClass = labels[c];
                    maxClass = dequantizedRows[i][5 + c];
                }
            }

            float confidenceInClass = maxClass * confidence;
            if (confidence > DETECTION_THRESHOLD) {
                float xPos = dequantizedRows[i][0];
                float yPos = dequantizedRows[i][1];

                float w = dequantizedRows[i][2];
                float h = dequantizedRows[i][3];

                var rect =
                        new RectF(
                                Math.max(0, xPos - w / 2),
                                Math.max(0, yPos - h / 2),
                                Math.min(bitmap.getWidth() - 1, xPos + w / 2),
                                Math.min(bitmap.getHeight() - 1, yPos + h / 2));
                detections.add(new Recognition("" + i, detectedClass,
                        confidenceInClass, rect));
            }
        }

        return USE_NON_MAXIUM_SUPPRESSION ? nms(detections) : detections;

    }


    public synchronized ArrayList<Classifier.Recognition> findGameElementsOnMat(Mat frame) {
        Imgproc.resize(frame, frame, new Size(416, 416));
        Core.rotate(frame, frame, Core.ROTATE_90_CLOCKWISE);
        var bmp = Bitmap.createBitmap(frame.cols(), frame.rows(), Bitmap.Config.ARGB_8888);
        org.opencv.android.Utils.matToBitmap(frame, bmp);

        var detections = recognizeImage(bmp);

        if(!USE_NON_MAXIUM_SUPPRESSION) {
            var filtered = new ArrayList<Classifier.Recognition>();
            outer:
            for (var detection : detections) {
                if (new RectF(0, 0, 416, 416).contains(detection.getLocation())) {
                    for (var previous : filtered)
                        if (Math.hypot(detection.getLocation().centerX() - previous.getLocation().centerX(), detection.getLocation().centerY() - previous.getLocation().centerY()) < CLOSENESS_THRESHOLD)
                            continue outer;

                    filtered.add(detection);
                }
            }

            return filtered;
        } else
            return detections;
    }

}

