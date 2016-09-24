package com.parrot.sdksample.drone;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;
import android.widget.TextView;

import com.parrot.sdksample.activity.MiniDroneActivity;

public class SensorController implements SensorEventListener {

    private MiniDrone minidrone;

    private SensorManager mSensorManager;
    private Sensor mSensor;

    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private final float EPSILON = 0.00001f;
    private float timestamp;

    private MiniDroneActivity miniDroneActivity;

    private Boolean status = false;

    private TextView x,y,z;

    public SensorController(
            MiniDrone drone,
            SensorManager manager,
            MiniDroneActivity activity,
            TextView X,
            TextView Y,
            TextView Z
    ){
        this.minidrone = drone;

        this.miniDroneActivity = activity;

        mSensorManager = manager;
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);

        this.x = X;
        this.y = Y;
        this.z = Z;

        //mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_FASTEST);

    }

    public Boolean getStatus(){
        return this.status;
    }

    public void toggleSensorStatus() {
        Log.d("SensorController","Toggle");
        this.status = !this.status;

        if(this.status){
            mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_UI);
        } else {
            mSensorManager.unregisterListener(this);
        }
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void onSensorChanged(SensorEvent event) {

        // This timestep's delta rotation to be multiplied by the current rotation
        // after computing it from the gyro sample data.

        if (timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;

            // Axis of the rotation sample, not normalized yet.
            float axisX = event.values[0];
            float axisY = event.values[1];
            float axisZ = event.values[2];



//
//            // Calculate the angular speed of the sample
            float omegaMagnitude = (float) Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

            // Normalize the rotation vector if it's big enough to get the axis
            // (that is, EPSILON should represent your maximum allowable margin of error)
            if (omegaMagnitude > EPSILON) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }
//
//            // Integrate around this axis with the angular speed by the timestep
//            // in order to get a delta rotation from this sample over the timestep
//            // We will convert this axis-angle representation of the delta rotation
//            // into a quaternion before turning it into the rotation matrix.
            float thetaOverTwo = omegaMagnitude * dT / 2.0f;
            float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
            float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
            deltaRotationVector[0] = sinThetaOverTwo * axisX;
            deltaRotationVector[1] = sinThetaOverTwo * axisY;
            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
            deltaRotationVector[3] = cosThetaOverTwo;

//            Log.d("Position", ""+ deltaRotationVector[0] + " - "+ deltaRotationVector[1] + " - "+ deltaRotationVector[2] + " - "+ deltaRotationVector[3]);

            this.x.setText(String.format("%f", axisX));
            this.y.setText(String.format("%f", axisY));
            this.z.setText(String.format("%f", axisZ));
        }
        timestamp = event.timestamp;
        float[] deltaRotationMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);

//        Log.d("VECTOR","-----------------------------------");
//        Log.d("VECTOR","["+deltaRotationMatrix[0] + " - "+deltaRotationMatrix[1] + " - "+deltaRotationMatrix[2]);
//        Log.d("VECTOR","["+deltaRotationMatrix[3] + " - "+deltaRotationMatrix[4] + " - "+deltaRotationMatrix[5]);
//        Log.d("VECTOR","["+deltaRotationMatrix[6] + " - "+deltaRotationMatrix[7] + " - "+deltaRotationMatrix[8]);

        // User code should concatenate the delta rotation we computed with the current rotation
        // in order to get the updated rotation.
        // rotationCurrent = rotationCurrent * deltaRotationMatrix;
    }
}