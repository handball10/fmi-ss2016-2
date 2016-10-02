package com.parrot.sdksample.drone;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

/**
 * Created by Florian Gümbel on 01.10.2016.
 */
public class AccelerationController implements SensorEventListener {


    private SensorManager mSensorManager;
    private Sensor mSensor;
    private MiniDrone mMiniDrone;
    private boolean active = false;
    private boolean isFlying = false;

    private final float[] gravity = new float[3];
    private final float[] linear_acceleration = new float[3];

    public AccelerationController(MiniDrone drone, SensorManager manager){
        mSensorManager = manager;
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mMiniDrone = drone;
        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_UI);

    }


    public void onSensorChanged(SensorEvent event){
        final float alpha = 0.8f;



        // Isolate the force of gravity with the low-pass filter.
        gravity[0] = alpha * gravity[0] + (1 - alpha) * event.values[0];
        gravity[1] = alpha * gravity[1] + (1 - alpha) * event.values[1];
        gravity[2] = alpha * gravity[2] + (1 - alpha) * event.values[2];

//        Log.d("ACC-RAW-X", ""+event.values[0]);
//        Log.d("ACC-RAW-Y", ""+event.values[1]);
//        Log.d("ACC-RAW-Z", ""+event.values[2]);
//
//        Log.d("ACC-RAW-GX", ""+gravity[0]);
//        Log.d("ACC-RAW-GY", ""+gravity[1]);
//        Log.d("ACC-RAW-GZ", ""+gravity[2]);
        // Remove the gravity contribution with the high-pass filter.
        linear_acceleration[0] = event.values[0] - gravity[0];
        linear_acceleration[1] = event.values[1] - gravity[1];
        linear_acceleration[2] = event.values[2] - gravity[2];

//        Log.d("ACC-X", ""+linear_acceleration[0]);
//        Log.d("ACC-Y", ""+linear_acceleration[1]);

        if(linear_acceleration[2] > 6) {
            //start();
            Log.d("ACC-Z", "" + linear_acceleration[2] + " - " +gravity[2]);
        }

        if(linear_acceleration[1] > 6){
            Log.d("ACC-Y", "" + linear_acceleration[1] + " - " +gravity[1]);
        }

        if(linear_acceleration[0] > 6){
            Log.d("ACC-X", "" + linear_acceleration[0] + " - " +gravity[0]);
        }
    }

    public void setActive(){
        active = true;
    }

    public void disableActive(){
        active = false;
    }

    private void start(){
        if(active && !isFlying){
            switch (mMiniDrone.getFlyingState()) {
                case ARCOMMANDS_MINIDRONE_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED:
                    mMiniDrone.takeOff();
                    isFlying = true;
                    break;
                case ARCOMMANDS_MINIDRONE_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_FLYING:
                case ARCOMMANDS_MINIDRONE_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_HOVERING:
                    mMiniDrone.land();
                    break;
                default:
            }

        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}
