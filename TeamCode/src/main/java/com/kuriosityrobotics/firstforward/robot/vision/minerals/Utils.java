package com.kuriosityrobotics.firstforward.robot.vision.minerals;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class Utils {

    /**
     * Memory-map the model file in Assets.
     */
    public static MappedByteBuffer loadModelFile(AssetManager assets, String modelFilename)
            throws IOException {
        AssetFileDescriptor fileDescriptor = assets.openFd(modelFilename);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    //    public static Bitmap scale(Context context, String filePath) {
//        AssetManager assetManager = context.getAssets();
//
//        InputStream istr;
//        Bitmap bitmap = null;
//        try {
//            istr = assetManager.open(filePath);
//            bitmap = BitmapFactory.decodeStream(istr);
//            bitmap = Bitmap.createScaledBitmap(bitmap, MainActivity.TF_OD_API_INPUT_SIZE, MainActivity.TF_OD_API_INPUT_SIZE, false);
//        } catch (IOException e) {
//            // handle exception
//            Log.e("getBitmapFromAsset", "getBitmapFromAsset: " + e.getMessage());
//        }
//
//        return bitmap;
//    }

}

