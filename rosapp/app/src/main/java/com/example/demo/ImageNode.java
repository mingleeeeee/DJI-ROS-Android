package com.example.demo;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.common.base.Preconditions;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.MessageBuffers;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;

public class ImageNode implements NodeMain {

    private static final java.lang.String  NODE_NAME = "img_node";
    private Timer imageTimer = null;
    private ImageTask imageTask = null;
    private ConnectedNode connectedNode = null;
    public static final int imgPERIOD = 33;
    private Publisher<CompressedImage> pubImage;
    private Publisher<CameraInfo> info_pub;
    private ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    private boolean isNeed = true;


    public synchronized boolean getNeed() {
        return isNeed;
    }
    public synchronized void setNeed(boolean isNeed) {
        this.isNeed = isNeed;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Log.d("ros_connected","connected!!!!!");
        this.connectedNode = connectedNode;

        pubImage = connectedNode.newPublisher("/image/compressed", CompressedImage._TYPE);
        info_pub = connectedNode.newPublisher("/image_info", CameraInfo._TYPE);

        if (imageTimer == null) {
            imageTask = new ImageTask();
            imageTimer = new Timer();
            imageTimer.schedule(imageTask, 0, imgPERIOD);
        }
    }

    @Override
    public void onShutdown(Node node) {
        if (null != imageTimer) {
            imageTimer.cancel();
            imageTimer.purge();
            imageTimer = null;
            imageTask = null;
        }
    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    private class ImageTask extends TimerTask {

        @Override
        public void run() {
            if (BaseDJIApplication.getProductInstance() != null ? BaseDJIApplication.getProductInstance().isConnected() : false) {
                if (MainActivity.getBit() != null) {
                    Bitmap temp = MainActivity.getBit().copy(MainActivity.getBit().getConfig(), true);
                    isNeed = false;
                    int w = 428;
                    int h = 240;
                    Bitmap resized = Bitmap.createScaledBitmap(temp, w, h, true);
                    temp.recycle();
                    temp = null;
                    ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
                    resized.compress(Bitmap.CompressFormat.JPEG, 100, byteArrayOutputStream);
                    byte[] pixels = byteArrayOutputStream.toByteArray();
                    resized.recycle();
                    try {
                        byteArrayOutputStream.flush();
                        byteArrayOutputStream.close();
                        byteArrayOutputStream = null;
                    } catch (IOException e) {
                        e.printStackTrace();
                    }

                    temp = null;
                    Preconditions.checkNotNull(pixels);
                    CompressedImage image = connectedNode.getTopicMessageFactory().newFromType(CompressedImage._TYPE);
                    CameraInfo cameraInfo = connectedNode.getTopicMessageFactory().newFromType(CameraInfo._TYPE);
                    cameraInfo.setWidth(428);
                    cameraInfo.setHeight(240);
                    //image.setHeight(resized.getHeight());
                    //image.setWidth(resized.getWidth());
                    //int x = resized.getRowBytes();
                    //image.setStep(x);
                    //image.setEncoding("rgba8");
                    String frameId = "android_camera";
                    image.setFormat("jpeg");
                    image.getHeader().setStamp(connectedNode.getCurrentTime());
                    image.getHeader().setFrameId(frameId);
                    cameraInfo.getHeader().setStamp(image.getHeader().getStamp());
                    cameraInfo.getHeader().setFrameId(image.getHeader().getFrameId());
                    //resized.recycle();
                    resized = null;
                    try {
                        stream.write(pixels);
                    } catch (IOException e) {
                        e.printStackTrace();
                        throw new RosRuntimeException(e);
                    }
                    cameraInfo.setDistortionModel("plumb_bob");
                    //distortion_coefficients
                    double[] d = {-0.008413, 0.040656, -0.002255, 0.001689, 0.000000};
                    cameraInfo.setD(d);
                    //camera_matrix
                    cameraInfo.setK(new double[]{657.4833, 0., 428.35019, 0., 663.50075, 233.10137, 0., 0., 1.});
                    cameraInfo.setR(new double[]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
                    cameraInfo.setP(new double[]{662.45563, 0., 429.30829, 0., 0., 668.35706, 231.50484, 0., 0., 0., 1., 0. });
                    image.setData(stream.buffer().copy());
                    stream.buffer().clear();
                    pubImage.publish(image);
                    info_pub.publish(cameraInfo);
                    isNeed = true;
                    //image = null;
                    //System.gc();
                    //isUpdate = false;
                }
            }
        }
    }
}
