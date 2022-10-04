package com.example.demo;

import android.Manifest;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.SurfaceTexture;
import android.hardware.usb.UsbManager;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;


import org.ros.android.RosActivity;
import org.ros.internal.message.MessageBuffers;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

import dji.common.camera.SettingsDefinitions;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.log.DJILog;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.sdkmanager.DJISDKInitEvent;
import dji.sdk.sdkmanager.DJISDKManager;
import sensor_msgs.CompressedImage;
import org.jboss.netty.buffer.ChannelBufferOutputStream;

public class MainActivity extends RosActivity implements TextureView.SurfaceTextureListener{

    private static final String TAG = MainActivity.class.getSimpleName();
    private static final String[] REQUIRED_PERMISSION_LIST = new String[] {
            Manifest.permission.VIBRATE, // Gimbal rotation
            Manifest.permission.INTERNET, // API requests
            Manifest.permission.ACCESS_WIFI_STATE, // WIFI connected products
            Manifest.permission.ACCESS_COARSE_LOCATION, // Maps
            Manifest.permission.ACCESS_NETWORK_STATE, // WIFI connected products
            Manifest.permission.ACCESS_FINE_LOCATION, // Maps
            Manifest.permission.CHANGE_WIFI_STATE, // Changing between WIFI and USB connection
            Manifest.permission.WRITE_EXTERNAL_STORAGE, // Log files
            Manifest.permission.BLUETOOTH, // Bluetooth connected products
            Manifest.permission.BLUETOOTH_ADMIN, // Bluetooth connected products
            Manifest.permission.READ_EXTERNAL_STORAGE, // Log files
            Manifest.permission.READ_PHONE_STATE, // Device UUID accessed upon registration
            Manifest.permission.RECORD_AUDIO // Speaker accessory
    };
    private static final int REQUEST_PERMISSION_CODE = 12345;
    private List<String> missingPermission = new ArrayList<>();
    private AtomicBoolean isRegistrationInProgress = new AtomicBoolean(false);
    private int lastProcess = -1;
    private boolean isVideoRecording = false;
    private Handler mHander = new Handler();
    private BaseComponent.ComponentListener mDJIComponentListener = new BaseComponent.ComponentListener() {

        @Override
        public void onConnectivityChange(boolean isConnected) {
            Log.d(TAG, "onComponentConnectivityChanged: " + isConnected);
            notifyStatusChange();
        }
    };
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    protected DJICodecManager mCodecManager = null;
    protected TextureView mVideoSurface = null;

    private TextView textView, ip;
    private Switch connectedSwitch, connectedROSSwitch;
    private ProgressBar connectedProgress;

    private DjiRosDriverNode rosDriver = new DjiRosDriverNode();
    private IsRosConnectedNode isRosConnectedNode = new IsRosConnectedNode();
    private ImageNode imageNode = new ImageNode();
    private Timer checkConnectionToDroneTimer = null;
    private CheckConnectionToDroneTask checkConnectionToDroneTask;
    private Button btn;
    private Button videobtn;
    private boolean isDroneConnected = false;
    private Node mapNode;
    private Publisher<CompressedImage> imgPub;
    private static Bitmap bit = null;
    private boolean isUpdate = false;
    private ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    //bug
    

    public MainActivity() {
        //super("ros dji driver", "ros dji driver", URI.create("http://192.168.129.113:11311"));
        super("ros dji driver", "ros dji driver");
    }
    //homeip 192.168.129.113
    public static synchronized Bitmap getBit() {
        return bit;
    }

    @SuppressWarnings("unchecked")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        checkAndRequestPermissions();
        BaseDJIApplication.getEventBus().register(this);
        // When the compile and target version is higher than 22, please request the following permission at runtime to ensure the SDK works well.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            //checkAndRequestPermissions();
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.VIBRATE,
                    Manifest.permission.INTERNET, Manifest.permission.ACCESS_WIFI_STATE,
                    Manifest.permission.WAKE_LOCK, Manifest.permission.ACCESS_COARSE_LOCATION,
                    Manifest.permission.ACCESS_NETWORK_STATE, Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.CHANGE_WIFI_STATE, Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS,
                    Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.SYSTEM_ALERT_WINDOW,
                    Manifest.permission.READ_PHONE_STATE,},1);
        }
        //LayoutInflater layoutInflater = (LayoutInflater) getApplicationContext().getSystemService(Service.LAYOUT_INFLATER_SERVICE);
        //View view = getLayoutInflater().inflate(R.layout.activity_main, null);
        setContentView(R.layout.activity_main);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
        initUI();

        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (mCodecManager != null) {
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };

        Camera camera = FPVDemoApplication.getCameraInstance();

        if (camera != null) {

            camera.setSystemStateCallback(new SystemState.Callback() {
                @Override
                public void onUpdate(SystemState cameraSystemState) {
                    if (null != cameraSystemState){
                        isVideoRecording = cameraSystemState.isRecording();
                    }
                }
            });

        }

        //startLiveShow();
    }

    private void initUI() {
        ip = (TextView)findViewById(R.id.textView2);

        textView = (TextView) findViewById(R.id.textDebug);
        mVideoSurface = (TextureView)findViewById(R.id.video_previewer_surface);
        //primaryVideoFeedView = (VideoFeedView) findViewById(R.id.video_view_primary_video_feed);
        connectedSwitch = (Switch) findViewById(R.id.switch_dron_conn);
        connectedROSSwitch = (Switch) findViewById(R.id.switch_ros_conn);
        connectedProgress = (ProgressBar) findViewById(R.id.connected_progressBar);

        checkConnectionToDroneTask = new CheckConnectionToDroneTask();
        checkConnectionToDroneTimer = new Timer();
        checkConnectionToDroneTimer.schedule(checkConnectionToDroneTask,0,500);

        btn = (Button) findViewById(R.id.button_start_vsticks);
        btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                rosDriver.StartVirtualSticks();
            }
        });
        videobtn = (Button) findViewById(R.id.button_start_rec);
        videobtn.setOnClickListener(new View.OnClickListener(){

            @Override
            public void onClick(View view) {
              if (isVideoRecording == false) {
                  switch_mode();
                  startRecording();
              }
              else{
                  stopRecord();
              }
            }
        });
        //Initialize DJI SDK Manager
        //mHandler = new Handler(Looper.getMainLooper());
        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        // Check for granted permission and remove from missing list
        if (requestCode == REQUEST_PERMISSION_CODE) {
            for (int i = grantResults.length - 1; i >= 0; i--) {
                if (grantResults[i] == PackageManager.PERMISSION_GRANTED) {
                    missingPermission.remove(permissions[i]);
                }
            }
        }
        // If there is enough permission, we will start the registration
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else {
            Toast.makeText(getApplicationContext(), "Missing permissions!!!", Toast.LENGTH_LONG).show();
        }
    }

    private void onProductChange() {
        initPreviewer();
    }

    @Override
    public void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        initPreviewer();
        onProductChange();
    }

    @Override
    public void onPause() {
        Log.e(TAG, "onPause");
        //uninitPreviewer();
        super.onPause();
    }

    @Override
    public void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    public void onReturn(View view){
        Log.e(TAG, "onReturn");
        this.finish();
    }

    @Override
    protected void onDestroy() {
        BaseDJIApplication.getEventBus().unregister(this);
        uninitPreviewer();
        super.onDestroy();
    }

    private void initPreviewer() {
        BaseProduct product = FPVDemoApplication.getProductInstance();
        if (product == null || !product.isConnected()) {}
        else{
            if (null != mVideoSurface){
                mVideoSurface.setSurfaceTextureListener(this);
            }
            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }
        }
    }

    private void uninitPreviewer() {
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(null);
        }
    }

    private void checkAndRequestPermissions() {
        // Check for permissions
        for (String eachPermission : REQUIRED_PERMISSION_LIST) {
            if (ContextCompat.checkSelfPermission(this, eachPermission) != PackageManager.PERMISSION_GRANTED) {
                missingPermission.add(eachPermission);
            }
        }
        // Request for missing permissions
        if (missingPermission.isEmpty()) {
            startSDKRegistration();
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this,
                    missingPermission.toArray(new String[missingPermission.size()]),
                    REQUEST_PERMISSION_CODE);
        }
    }

    private void switch_mode()
    {
        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null){
            camera.setMode(SettingsDefinitions.CameraMode.RECORD_VIDEO, new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError ==null){
                        showToast("Switch Camera Mode Recording");
                    }
                    else{
                        showToast(djiError.getDescription());
                    }
                }
            });
        }
    }

    private void startRecording()
    {
        final Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null){
            camera.startRecordVideo(new CommonCallbacks.CompletionCallback() {
                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null) {
                        showToast("Record video: success");
                    }else {
                        showToast(djiError.getDescription());
                    }
                }
            });
        }
    }

    private void stopRecord(){

        Camera camera = FPVDemoApplication.getCameraInstance();
        if (camera != null) {
            camera.stopRecordVideo(new CommonCallbacks.CompletionCallback() {

                @Override
                public void onResult(DJIError djiError) {
                    if (djiError == null) {
                        showToast("Stop recording: success");
                    } else {
                        showToast(djiError.getDescription());
                    }
                }
            }); // Execute the stopRecordVideo API
        }
    }

    @Override
    protected void onNewIntent(@NonNull Intent intent) {
        String action = intent.getAction();
        if (UsbManager.ACTION_USB_ACCESSORY_ATTACHED.equals(action)) {
            Intent attachedIntent = new Intent();
            attachedIntent.setAction(DJISDKManager.USB_ACCESSORY_ATTACHED);
            sendBroadcast(attachedIntent);
        }
    }

    private void startSDKRegistration() {
        if (isRegistrationInProgress.compareAndSet(false, true)) {
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    DJISDKManager.getInstance().registerApp(MainActivity.this.getApplicationContext(), new DJISDKManager.SDKManagerCallback() {
                        @Override
                        public void onRegister(DJIError djiError) {
                            if (djiError == DJISDKError.REGISTRATION_SUCCESS) {
                                DJILog.e("App registration", DJISDKError.REGISTRATION_SUCCESS.getDescription());
                                DJISDKManager.getInstance().startConnectionToProduct();
                            }

                            Log.v(TAG, djiError.getDescription());
                        }
                        @Override
                        public void onProductDisconnect() {
                            Log.d(TAG, "onProductDisconnect");
                            notifyStatusChange();
                        }
                        @Override
                        public void onProductConnect(BaseProduct baseProduct) {
                            Log.d(TAG, String.format("onProductConnect newProduct:%s", baseProduct));
                            notifyStatusChange();
                        }
                        @Override
                        public void onProductChanged(BaseProduct baseProduct) {

                        }
                        @Override
                        public void onComponentChange(BaseProduct.ComponentKey componentKey,
                                                      BaseComponent oldComponent,
                                                      BaseComponent newComponent) {
                            if (newComponent != null) {
                                newComponent.setComponentListener(mDJIComponentListener);
                            }
                            Log.d(TAG,
                                    String.format("onComponentChange key:%s, oldComponent:%s, newComponent:%s",
                                            componentKey,
                                            oldComponent,
                                            newComponent));

                            notifyStatusChange();
                        }

                        @Override
                        public void onInitProcess(DJISDKInitEvent djisdkInitEvent, int i) {

                        }

                        @Override
                        public void onDatabaseDownloadProgress(long current, long total) {
                            int process = (int) (100 * current / total);
                            if (process == lastProcess) {
                                return;
                            }
                            lastProcess = process;
                        }
                    });
                }
            });
        }
    }

    public void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }

    private void notifyStatusChange() {
        BaseDJIApplication.getEventBus().post(new ConnectivityChangeEvent());
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
      Log.e("TAG", "onSurfaceTextureAvailable");
      if (mCodecManager == null) {
          mCodecManager = new DJICodecManager(this, surface, width, height);
      }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        Log.e("TAG", "onSurfaceTextureSizeChanged");
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        Log.e("TAG", "onSurfaceTextureDestroyed");
        if (mCodecManager != null){
            mCodecManager.cleanSurface();
            mCodecManager = null;
        }
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        if(imageNode.getNeed() == true){
            bit = mVideoSurface.getBitmap();
        }
        else {
            //bit.recycle();
            bit = null;
        }

        //final Bitmap image = mVideoSurface.getBitmap();
        //bit = mVideoSurface.getBitmap();
        /*bit = image.copy(image.getConfig(), true);
        if (isUpdate == false && bit == null) {
            bit = image.copy(image.getConfig(), true);
            bit = Bitmap.createScaledBitmap(bit, (int)bit.getWidth()/2, (int)bit.getHeight()/2, true);
            isUpdate = true;
        }
        else {
            bit = image.copy(image.getConfig(), true);
            bit = Bitmap.createScaledBitmap(bit, (int)bit.getWidth()/2, (int)bit.getHeight()/2, true);
        }*/
    }

    private class CheckConnectionToDroneTask extends TimerTask {

        @Override
        public void run() {
            final boolean isDroneConnected = BaseDJIApplication.getProductInstance() != null ? BaseDJIApplication.getProductInstance().isConnected() : false;
            final boolean isROSConnected = isRosConnectedNode.IsConnected();
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                   connectedSwitch.setChecked(isDroneConnected);
                   connectedROSSwitch.setChecked(isROSConnected);
                   //btn.setEnabled(isDroneConnected);
                   connectedProgress.setVisibility(isDroneConnected && isROSConnected ? View.GONE: View.VISIBLE);
                }
            });
        }
    }

    public static class ConnectivityChangeEvent {
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        Log.d("TAG", getRosHostname());
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        ip.setText(getMasterUri().toString());
        nodeConfiguration.setMasterUri(getMasterUri());
        //ip.setText(nodeConfiguration.getMasterUri().toString());
        nodeMainExecutor.execute(isRosConnectedNode, nodeConfiguration);
        nodeMainExecutor.execute(rosDriver, nodeConfiguration);
        nodeMainExecutor.execute(imageNode, nodeConfiguration);

        /*nodeMainExecutor.execute(new NodeMain() {
            @Override
            public GraphName getDefaultNodeName() {
                return GraphName.of("ros_test");
            }

            @Override
            public void onStart(ConnectedNode connectedNode) {
                final Publisher<CompressedImage> pub =  connectedNode.newPublisher("/image/compressed", CompressedImage._TYPE);
                final Publisher<CameraInfo> info_pub = connectedNode.newPublisher("/image_info", CameraInfo._TYPE);
                connectedNode.executeCancellableLoop(new CancellableLoop() {
                    @Override
                    protected void loop() throws InterruptedException {
                        if (BaseDJIApplication.getProductInstance() != null ? BaseDJIApplication.getProductInstance().isConnected() : false){
                              if (bit != null) {
                                  //Bitmap copy1 = bit.copy(bit.getConfig(), true);
                                  Bitmap resized = Bitmap.createScaledBitmap(bit, (int) bit.getWidth() / 4, (int) bit.getHeight() / 4, true);
                                  //Log.d("TAG", (int) bit.getWidth() / 2 + "," + (int) bit.getHeight() / 2);
                                  //Toast.makeText("TAG", bit.getWidth() / 2 + "," + (int) bit.getHeight() / 2, Toast.LENGTH_SHORT).show();
                                  //int bytes = resized.getByteCount();
                                  //ByteBuffer buffer = ByteBuffer.allocate(bytes);
                                  //resized.copyPixelsToBuffer(buffer); // Move the byte data to the buffer
                                  //byte[] temp = buffer.array(); // Get the underlying array containing the data.

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

                                  byte[] pixels = new byte[(temp.length / 4) * 3]; // Allocate for 3 byte BGR

                                  // Copy pixels into place
                                  for (int i = 0; i < (temp.length / 4); i++) {
                                      pixels[i * 3] = temp[i * 4];     // B
                                      pixels[i * 3 + 1] = temp[i * 4 + 1]; // G
                                      pixels[i * 3 + 2] = temp[i * 4 + 2]; // R
                                      // Alpha is discarded
                                  }

                                  temp = null;
                                  Preconditions.checkNotNull(pixels);
                                  CompressedImage image = connectedNode.getTopicMessageFactory().newFromType(CompressedImage._TYPE);
                                  //image.setHeight(resized.getHeight());
                                  //image.setWidth(resized.getWidth());
                                  //int x = resized.getRowBytes();
                                  //image.setStep(x);
                                  //image.setEncoding("rgba8");
                                  image.setFormat("jpeg");

                                  String frameId = "android_camera";
                                  image.getHeader().setStamp(connectedNode.getCurrentTime());
                                  image.getHeader().setFrameId(frameId);
                                  //resized.recycle();
                                  resized = null;
                                  try {
                                      stream.write(pixels);
                                  } catch (IOException e) {
                                      e.printStackTrace();
                                      throw new RosRuntimeException(e);
                                  }
                                  image.setData(stream.buffer().copy());
                                  Thread.sleep(50);
                                  stream.buffer().clear();
                                  bit.recycle();
                                  bit = null;
                                  pub.publish(image);
                                  //image = null;
                                  System.gc();
                                  //isUpdate = false;

                              }
                        }
                    }
                });
            }

            @Override
            public void onShutdown(Node node) {

            }

            @Override
            public void onShutdownComplete(Node node) {

            }

            @Override
            public void onError(Node node, Throwable throwable) {

            }
        }, nodeConfiguration);*/
    }

}
