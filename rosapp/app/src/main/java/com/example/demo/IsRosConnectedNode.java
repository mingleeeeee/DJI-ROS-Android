package com.example.demo;

import android.util.Log;

import org.ros.internal.node.client.MasterClient;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import java.util.Timer;
import java.util.TimerTask;

public class IsRosConnectedNode implements NodeMain {

    private static final String NODE_NAME = "ros_connected";
    private Timer connectionTimer = null;
    private ConnectionTask connectionTask = null;
    private ConnectedNode connectedNode = null;
    private boolean isConnected = false;

    public boolean IsConnected()
    {
        return isConnected;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Log.d("ros_connected","connected!!!!!");

        this.connectedNode = connectedNode;

        if (connectionTimer == null) {
            connectionTask = new ConnectionTask();
            connectionTimer = new Timer();
            connectionTimer.schedule(connectionTask, 0, 2000);
        }

    }

    @Override
    public void onShutdown(Node node) {
        isConnected = false;
        Log.d("ros_connected","shutdown!!!!!");

        if (null != connectionTimer) {
            connectionTimer.cancel();
            connectionTimer.purge();
            connectionTimer = null;
            connectionTask = null;
        }
    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {
        isConnected = false;
        Log.e("ros_connected","error!!!!!");
    }

    private class ConnectionTask extends TimerTask {

        @Override
        public void run() {

            if (connectedNode != null) {

                try {
                    MasterClient masterClient = new MasterClient(connectedNode.getMasterUri());
                    masterClient.getUri(GraphName.of("android/master_chooser_activity"));

                    isConnected = true;
                }
                catch (Exception e) {
                    isConnected = false;
                }
            }
            else {
                isConnected = false;
            }
        }
    }
}
