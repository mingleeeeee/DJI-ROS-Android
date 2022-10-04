package com.example.demo;

import androidx.annotation.NonNull;

import org.ros.exception.ServiceException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Timer;
import java.util.TimerTask;

import dji.common.battery.BatteryState;
import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.FlightOrientationMode;
import dji.common.flightcontroller.imu.IMUState;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.CommonCallbacks;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.products.Aircraft;
import std_srvs.EmptyRequest;
import std_srvs.EmptyResponse;

public class DjiRosDriverNode implements NodeMain {

    public static final int PERIOD = 100;
    public static final int STATUS_PERIOD = 1000;

    private static final float M_PI = 3.14159f;
    private static final java.lang.String NODE_NAME = "dji_ros_driver";
    private static final java.lang.String TAKE_OFF_CMD = "takeoff";
    private static final java.lang.String LAND_CMD = "land";
    private static final java.lang.String ROTATE_CLOCKWISE_CMD = "rotate_cw";
    private static final java.lang.String ROTATE_COUNTERCLOCKWISE_CMD = "rotate_ccw";
    private static final java.lang.String STOP_CMD = "stop";
    private static final java.lang.String START_CMD = "start";
    //private static final java.lang.String GIMBAL_UP_CMD = "gimbal_up";
    //private static final java.lang.String GIMBAL_DOWN_CMD = "gimbal_down";
    private static final java.lang.String GIMBAL_ZERO_CMD = "gimbal_zero";
    private static final java.lang.String GIMBAL_NINE_CMD = "gimbal_90";
    private static final java.lang.String GIMBAL_THREE_CMD = "gimbal_30";

    private static final java.lang.String BASE_TOPIC_NAME = "/flight_commands";
    private static final java.lang.String CMD_VEL_TOPIC_NAME = "/cmd_vel";
    private static final java.lang.String DONE_TOPIC_NAME = "done";
    private static final java.lang.String STATUS_TOPIC_NAME = "/dji/status";

    private Timer sendVirtualStickDataTimer = null;
    private SendVirtualStickDataTask sendVirtualStickDataTask = null;
    private Timer droneStatusTimer = null;
    private DroneStatusTask droneStatusTask = null;
    private Timer gimbalTimer = null;
    private SendGimbalDataTask gimbal_task = null;

    // Movement command values
    private float pitch = 0f;
    private float roll = 0f;
    private float yaw = 0f;
    private float throttle = 0f;

    private boolean initialized = false;
    private float batteryLevelAvg = 0.0f;
    private boolean isConnected = false;
    private boolean isCallbackSetup = false;
    private boolean areMotorsOn = false;
    private boolean isFlying = false;
    private float altitude = 0.0f;
    private double latitude = 0.0f;
    private double longitude = 0.0f;
    private boolean landConfirmNeeded = false;
    private double pitch_degrees = 0.0;
    private double roll_degrees = 0.0;
    private double yaw_degrees = 0.0;
    private float pitchValue = 0.0f;
    private int headDirection = 0;
    private float att_yaw = 0.0f;
    private double home_lat = 0;
    private double home_lot = 0;
    private double bo_height = 0.0;
    private double gyroscopeValue = 0.0;
    private double accelerometer = 0.0;

    private java.lang.String commandsTopicName = BASE_TOPIC_NAME;
    private java.lang.String commandsMessageType = std_msgs.String._TYPE;
    private java.lang.String commandsResTopicName = BASE_TOPIC_NAME + "/" + DONE_TOPIC_NAME;

    private java.lang.String cmdvelTopicName = CMD_VEL_TOPIC_NAME;
    private java.lang.String cmdvelMessageType = geometry_msgs.Twist._TYPE;

    private java.lang.String djiStatusTopicName = STATUS_TOPIC_NAME;
    private java.lang.String djiStatusMessageType = std_msgs.String._TYPE;

    private Publisher<std_msgs.String> pubDjiStatus;
    private Publisher<std_msgs.Empty> pubResult;
    private Subscriber<geometry_msgs.Twist> subCmdvel;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverTakeOff;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverLand;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverRotate;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverStop;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverStart;
    //private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverGimbalUp;
    //private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverGimbalDown;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverGimbalZero;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverGimbalNine;
    private ServiceServer<std_srvs.EmptyRequest, std_srvs.EmptyResponse> serverGimbalThree;
    //private Time now;

    public java.lang.String getCommandsTopicName() {
        return this.commandsResTopicName;
    }
    public java.lang.String getCommandsMessageType() {
        return this.commandsMessageType;
    }

    public void setCommandsTopicName(java.lang.String topicName) {
        this.commandsResTopicName = topicName;
    }
    public void setCommandsMessageType(java.lang.String messageType) {
        this.commandsMessageType = messageType;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    public void StopVirtualSticks() {
        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
        initializeIfNeeded(flightController);
        flightController.setVirtualStickModeEnabled(false, null);
    }
    public void StartVirtualSticks() {
        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
        initializeIfNeeded(flightController);
        flightController.setVirtualStickModeEnabled(false, null);
        flightController.setVirtualStickModeEnabled(true, null);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

        //Setting up publishers
        pubDjiStatus = connectedNode.newPublisher(djiStatusTopicName, djiStatusMessageType);
        pubResult = connectedNode.newPublisher(commandsResTopicName, std_msgs.Empty._TYPE);

        subCmdvel = connectedNode.newSubscriber(this.cmdvelTopicName, this.cmdvelMessageType);
        subCmdvel.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
            @Override
            public void onNewMessage(geometry_msgs.Twist o) {
                // save commands in global fields which will be used in for the virtual sticks
                //Log.d("TAG", o.toString());
                //geometry_msgs.Twist message = (geometry_msgs.Twist) o;
                throttle = (float) o.getLinear().getZ(); // Movement along the Z axis
                pitch = (float) o.getLinear().getY(); // Movement along the y axis
                roll = (float) o.getLinear().getX(); // Movement along the x axis
                yaw = ((float) o.getAngular().getZ() * 180) / M_PI; // Convert to degrees. ROS works with radians...
                //now = Time.fromMillis(System.currentTimeMillis());
                //currentReceiveTime = System.currentTimeMillis()/1000;
                //Log.d("Tag",currentReceiveTime.toString());
            }
        });

        serverTakeOff = connectedNode.newServiceServer(
                commandsTopicName + "/" + TAKE_OFF_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<std_srvs.EmptyRequest, std_srvs.EmptyResponse>() {
                    @Override
                    public void build(std_srvs.EmptyRequest empty, std_srvs.EmptyResponse empty2) throws ServiceException {
                        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                        initializeIfNeeded(flightController);
                        flightController.startTakeoff(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                pubResult.publish(pubResult.newMessage());
                                StartVirtualSticks();
                            }
                        });
                    }
                });

        serverLand = connectedNode.newServiceServer(
                commandsTopicName + "/" + LAND_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<std_srvs.EmptyRequest, std_srvs.EmptyResponse>() {
                    @Override
                    public void build(std_srvs.EmptyRequest empty, std_srvs.EmptyResponse empty2) throws ServiceException {
                        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                        initializeIfNeeded(flightController);

                        flightController.startLanding(new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                pubResult.publish(pubResult.newMessage());
                            }
                        });
                    }
                });

        serverRotate = connectedNode.newServiceServer(
                commandsTopicName + "/" + ROTATE_CLOCKWISE_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<std_srvs.EmptyRequest, std_srvs.EmptyResponse>() {
                    @Override
                    public void build(std_srvs.EmptyRequest empty, std_srvs.EmptyResponse empty2) throws ServiceException {
                        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                        initializeIfNeeded(flightController);
                        if (isFlying) {
                            yaw = -20f;
                        }
                    }
                });

        serverStop = connectedNode.newServiceServer(
                commandsTopicName + "/" + STOP_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(std_srvs.EmptyRequest empty, std_srvs.EmptyResponse empty2) throws ServiceException {
                        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                        initializeIfNeeded(flightController);

                        //Setting all control parameters to 0 and disable virtual sticks

                        pitch = 0f;
                        roll = 0f;
                        yaw = 0f;
                        throttle = 0f;
                        flightController.setVirtualStickModeEnabled(false, null);

                    }
                });

        serverStart = connectedNode.newServiceServer(
                commandsTopicName + "/" + START_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(std_srvs.EmptyRequest empty, std_srvs.EmptyResponse empty2) throws ServiceException {
                        final FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                        initializeIfNeeded(flightController);

                        //Setting all control parameters to 0 and disable virtual sticks

                        pitch = 0f;
                        roll = 0f;
                        yaw = 0f;
                        throttle = 0f;
                        flightController.setVirtualStickModeEnabled(false, null);
                        flightController.setVirtualStickModeEnabled(true, null);
                    }
                });

        /*serverGimbalUp = connectedNode.newServiceServer(
                commandsTopicName + "/" + GIMBAL_UP_CMD, Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(EmptyRequest emptyRequest, EmptyResponse emptyResponse) throws ServiceException {

                        pitchValue = 10.0f;
                        int now = connectedNode.getCurrentTime().secs;
                        while (connectedNode.getCurrentTime().secs - now < 2.0)
                        {}
                        pitchValue = 0.0f;
                    }
                }
        );*/

        /*serverGimbalDown = connectedNode.newServiceServer(
                commandsTopicName + "/" + GIMBAL_DOWN_CMD, Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(EmptyRequest emptyRequest, EmptyResponse emptyResponse) throws ServiceException {
                        pitchValue = -10.0f;
                        int now = connectedNode.getCurrentTime().secs;
                        while (connectedNode.getCurrentTime().secs - now < 2.0)
                        {}
                        pitchValue = 0.0f;
                    }
                }
        );*/

        serverGimbalZero = connectedNode.newServiceServer(
                commandsTopicName + "/" + GIMBAL_ZERO_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(EmptyRequest emptyRequest, EmptyResponse emptyResponse) throws ServiceException {
                        pitchValue = 0.0f;
                    }
                }
        );

        serverGimbalNine= connectedNode.newServiceServer(
                commandsTopicName + "/" + GIMBAL_NINE_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(EmptyRequest emptyRequest, EmptyResponse emptyResponse) throws ServiceException {
                        pitchValue = -90.0f;
                    }
                }
        );

        serverGimbalThree= connectedNode.newServiceServer(
                commandsTopicName + "/" + GIMBAL_THREE_CMD, std_srvs.Empty._TYPE, new ServiceResponseBuilder<EmptyRequest, EmptyResponse>() {
                    @Override
                    public void build(EmptyRequest emptyRequest, EmptyResponse emptyResponse) throws ServiceException {
                        pitchValue = -30.0f;
                    }
                }
        );

        //Setting up listeners
        //subCmdvel = connectedNode.newSubscriber(this.cmdvelTopicName, this.cmdvelMessageType);


        //Running threads
        runVirtualStickThread();
        runDroneStatusThread();
        runGimbalThread();
    }

    @Override
    public void onShutdown(Node node) {
        if (null != sendVirtualStickDataTimer) {
            sendVirtualStickDataTimer.cancel();
            sendVirtualStickDataTimer.purge();
            sendVirtualStickDataTimer = null;
            sendVirtualStickDataTask = null;
        }

        if (null != droneStatusTimer) {
            droneStatusTimer.cancel();
            droneStatusTimer.purge();
            droneStatusTimer = null;
            droneStatusTask = null;
        }

        if (null != gimbalTimer) {
            gimbalTimer.cancel();
            gimbalTimer.purge();
            gimbalTimer = null;
            gimbal_task = null;
        }
    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    private void runGimbalThread() {
        if (null == gimbalTimer) {
            gimbal_task = new SendGimbalDataTask();
            gimbalTimer = new Timer();
            gimbalTimer.schedule(gimbal_task, 0, PERIOD);
        }
    }

    private void runVirtualStickThread() {
        if (null == sendVirtualStickDataTimer) {
            sendVirtualStickDataTask = new SendVirtualStickDataTask();
            sendVirtualStickDataTimer = new Timer();
            sendVirtualStickDataTimer.schedule(sendVirtualStickDataTask, 0, PERIOD);
        }
    }

    private void runDroneStatusThread() {
        if (null == droneStatusTimer) {
            droneStatusTask = new DroneStatusTask();
            droneStatusTimer = new Timer();
            droneStatusTimer.schedule(droneStatusTask, 0, STATUS_PERIOD);
        }
    }

    private void initializeIfNeeded(FlightController flightController) {

        if (!initialized) {

            flightController.setVirtualStickModeEnabled(true, null);
            flightController.setNoviceModeEnabled(false, null);
            flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
            flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
            flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
            flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
            flightController.getFlightAssistant().setCollisionAvoidanceEnabled(false, null);
            flightController.getFlightAssistant().setActiveObstacleAvoidanceEnabled(false, null);
            flightController.setFlightOrientationMode(FlightOrientationMode.AIRCRAFT_HEADING, null);
            flightController.getFlightAssistant().setLandingProtectionEnabled(false, null);

            initialized = true;
        }
    }

    private class SendVirtualStickDataTask extends TimerTask {

        @Override
        public void run() {
            //Long tsLong =
            //String ts = tsLong.toString();

            if (BaseDJIApplication.getProductInstance() != null) {
                FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                initializeIfNeeded(flightController);

                /*if (Time.fromMillis(System.currentTimeMillis()).secs - now.secs > 1.5f) {
                    pitch = 0.0f;
                    roll = 0.0f;
                    yaw = 0.0f;
                    throttle = 0.0f;
                }*/
                //use the global fields and send as virtual sticks params.
                flightController.sendVirtualStickFlightControlData(new FlightControlData(pitch,
                                roll,
                                yaw,
                                throttle),
                        new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                            }
                        });
            }
        }
    }

    private class SendGimbalDataTask extends TimerTask {

        @Override
        public void run() {
            if ( ModuleVerificationUtil.isGimbalModuleAvailable()) {
                BaseDJIApplication.getProductInstance().getGimbal().
                        rotate(new Rotation.Builder().pitch(pitchValue)
                                                     .yaw(0)
                                                    .mode(RotationMode.ABSOLUTE_ANGLE)
                                                    .roll(Rotation.NO_ROTATION)
                                                    .time(1.5)
                                                    .build(), new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {

                    }
                });
            }
        }
    }


    private class DroneStatusTask extends TimerTask {

        @Override
        public void run() {
            if (BaseDJIApplication.getProductInstance() != null) {
                isConnected = BaseDJIApplication.getProductInstance().isConnected();

                if (!isCallbackSetup) {
                    BaseDJIApplication.getProductInstance().getBattery().setStateCallback(new BatteryState.Callback() {
                        @Override
                        public void onUpdate(BatteryState batteryState) {
                            batteryLevelAvg = batteryState.getChargeRemainingInPercent();
                        }
                    });


                    ((Aircraft) BaseDJIApplication.getProductInstance()).getFlightController().setIMUStateCallback(new IMUState.Callback() {
                        @Override
                        public void onUpdate(IMUState imuState) {
                            gyroscopeValue = imuState.getGyroscopeValue();
                            accelerometer = imuState.getAccelerometerValue();
                        }
                    });

                    ((Aircraft) BaseDJIApplication.getProductInstance()).getFlightController().setStateCallback(new FlightControllerState.Callback() {
                        @Override
                        public void onUpdate(FlightControllerState flightControllerState) {
                            areMotorsOn = flightControllerState.areMotorsOn();
                            isFlying = flightControllerState.isFlying();
                            altitude = flightControllerState.getAircraftLocation().getAltitude();
                            latitude = flightControllerState.getAircraftLocation().getLatitude();
                            longitude = flightControllerState.getAircraftLocation().getLongitude();
                            landConfirmNeeded = false;//flightControllerState.isLandingConfirmationNeeded();
                            headDirection = flightControllerState.getAircraftHeadDirection();
                            att_yaw = (float) flightControllerState.getAttitude().yaw;
                            home_lat = flightControllerState.getHomeLocation().getLatitude();
                            home_lot = flightControllerState.getHomeLocation().getLongitude();
                            bo_height = flightControllerState.getUltrasonicHeightInMeters();


                            if (landConfirmNeeded) {
                                FlightController flightController = ((Aircraft) (BaseDJIApplication.getProductInstance())).getFlightController();
                                flightController.confirmLanding(new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {
                                        pubResult.publish(pubResult.newMessage());
                                    }
                                });
                            }
                        }
                    });

                    BaseDJIApplication.getProductInstance().getGimbal().setStateCallback(new GimbalState.Callback() {
                        @Override
                        public void onUpdate(@NonNull GimbalState gimbalState) {
                            roll_degrees = gimbalState.getAttitudeInDegrees().getRoll();
                            pitch_degrees = gimbalState.getAttitudeInDegrees().getPitch();
                            yaw_degrees = gimbalState.getAttitudeInDegrees().getYaw();
                        }
                    });
                }
            }

            std_msgs.String msg = pubDjiStatus.newMessage();
            msg.setData("home_lat=" + home_lat + ";home_lot=" + home_lot + ";headDirection=" + headDirection + ";att_yaw=" + att_yaw + ";battery=" + batteryLevelAvg + ";isConnected=" + isConnected + ";areMotorsOn=" + areMotorsOn + ";isFlying=" + isFlying + ";lat=" + latitude + ";long=" + longitude + ";altitude=" + altitude + ";roll_degrees=" + roll_degrees  + ";pitch_degrees=" + pitch_degrees + ";yaw_degrees=" + yaw_degrees + ";bo_height=" + bo_height+";gyroscopeValue="+gyroscopeValue+";accelerometer="+accelerometer);
            pubDjiStatus.publish(msg);
        }
    }
}

