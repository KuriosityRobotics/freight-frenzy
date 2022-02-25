package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import android.graphics.Bitmap;
import android.graphics.RectF;

import java.util.List;

public interface Classifier {
    List<Recognition> recognizeImage(Bitmap bitmap);

    void enableStatLogging(final boolean debug);

    String getStatString();

    void close();

    void setNumThreads(int num_threads);

    void setUseNNAPI(boolean isChecked);

    float getObjThresh();

    class Recognition {
        private final String id;

        private final String detectionType;

        private final Float confidence;

        private RectF location;

        public Recognition(
                final String id, final String detectionType, final Float confidence, final RectF location) {
            this.id = id;
            this.detectionType = detectionType;
            this.confidence = confidence;
            this.location = location;
        }

        public String getId() {
            return id;
        }

        public String getDetectionType() {
            return detectionType;
        }

        public Float getConfidence() {
            return confidence;
        }

        public RectF getLocation() {
            return new RectF(location);
        }

        public void setLocation(RectF location) {
            this.location = location;
        }


        @Override
        public String toString() {
            String resultString = "";
            if (id != null) {
                resultString += "[" + id + "] ";
            }

            if (detectionType != null) {
                resultString += detectionType + " ";
            }

            if (confidence != null) {
                resultString += String.format("(%.1f%%) ", confidence * 100.0f);
            }

            if (location != null) {
                resultString += location + " ";
            }

            return resultString.trim();
        }
    }
}

