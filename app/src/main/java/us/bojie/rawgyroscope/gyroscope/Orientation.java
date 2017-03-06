package us.bojie.rawgyroscope.gyroscope;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;


/*
 * Gyroscope Explorer
 * Copyright (C) 2013-2015, Kaleb Kircher - Kircher Engineering, LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * An abstract class that provides an interface for classes that deal with
 * gyroscope integration and filters. Takes care of a lot of the boiler plate
 * code.
 * 
 * @author Kaleb
 *
 */
public abstract class Orientation implements OrientationInterface,
        SensorEventListener
{
	private final static String tag = Orientation.class.getSimpleName();

	protected static final float EPSILON = 0.000000001f;

	// private static final float NS2S = 1.0f / 10000.0f;
	// Nano-second to second conversion
	protected static final float NS2S = 1.0f / 1000000000.0f;

	private boolean calibratedGyroscopeEnabled = true;

	protected boolean meanFilterSmoothingEnabled = false;
	protected boolean isOrientationValidAccelMag = false;

	protected float dT = 0;

	protected float meanFilterTimeConstant = 0.2f;

	// angular speeds from gyro
	protected float[] vGyroscope = new float[3];

	// magnetic field vector
	protected float[] vMagnetic = new float[3];

	// accelerometer vector
	protected float[] vAcceleration = new float[3];

	// accelerometer and magnetometer based rotation matrix
	protected float[] rmOrientationAccelMag = new float[9];

	protected float[] vOrientationAccelMag = new float[3];

	protected long timeStampGyroscope = 0;
	protected long timeStampGyroscopeOld = 0;

	private Context context;

	private MeanFilterSmoothing meanFilterAcceleration;
	private MeanFilterSmoothing meanFilterMagnetic;
	private MeanFilterSmoothing meanFilterGyroscope;

	// We need the SensorManager to register for Sensor Events.
	protected SensorManager sensorManager;

	public Orientation(Context context)
	{
		this.context = context;
		this.sensorManager = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);

		initFilters();
	}

	@Override
	public void onSensorChanged(SensorEvent event)
	{
		if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
		{
			// Get a local copy of the raw magnetic values from the device
			// sensor.
			System.arraycopy(event.values, 0, this.vAcceleration, 0,
					this.vGyroscope.length);

			if (meanFilterSmoothingEnabled)
			{
				this.vAcceleration = meanFilterAcceleration
						.addSamples(this.vAcceleration);
			}

			// We fuse the orientation of the magnetic and acceleration sensor
			// based on acceleration sensor updates. It could be done when the
			// magnetic sensor updates or when they both have updated if you
			// want to spend the resources to make the checks.
			calculateOrientationAccelMag();
		}

		if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
		{
			// Get a local copy of the raw magnetic values from the device
			// sensor.
			System.arraycopy(event.values, 0, this.vMagnetic, 0,
					this.vGyroscope.length);

			if (meanFilterSmoothingEnabled)
			{
				this.vMagnetic = meanFilterMagnetic.addSamples(this.vMagnetic);
			}
		}

		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
		{
			System.arraycopy(event.values, 0, this.vGyroscope, 0,
					this.vGyroscope.length);

			if (meanFilterSmoothingEnabled)
			{
				this.vGyroscope = meanFilterGyroscope
						.addSamples(this.vGyroscope);
			}

			timeStampGyroscope = event.timestamp;

			onGyroscopeChanged();
		}

		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
		{
			System.arraycopy(event.values, 0, this.vGyroscope, 0,
					this.vGyroscope.length);

			if (meanFilterSmoothingEnabled)
			{
				this.vGyroscope = meanFilterGyroscope
						.addSamples(this.vGyroscope);
			}

			timeStampGyroscope = event.timestamp;

			onGyroscopeChanged();
		}

	}

	public void onPause()
	{
		sensorManager.unregisterListener(this);
	}

	public void onResume()
	{
		calibratedGyroscopeEnabled = true;
		meanFilterSmoothingEnabled = false;
		meanFilterTimeConstant = 0.5f;

		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_FASTEST);

		sensorManager.registerListener(this,
				sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
				SensorManager.SENSOR_DELAY_FASTEST);

		if (calibratedGyroscopeEnabled)
		{
			sensorManager.registerListener(this,
					sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
					SensorManager.SENSOR_DELAY_FASTEST);
		}
		else
		{
			if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.JELLY_BEAN_MR2)
			{
				sensorManager.registerListener(this, sensorManager
						.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED),
						SensorManager.SENSOR_DELAY_FASTEST);
			}
		}
	}

	protected void calculateOrientationAccelMag()
	{
		// To get the orientation vector from the acceleration and magnetic
		// sensors, we let Android do the heavy lifting. This call will
		// automatically compensate for the tilt of the compass and fail if the
		// magnitude of the acceleration is not close to 9.82m/sec^2. You could
		// perform these steps yourself, but in my opinion, this is the best way
		// to do it.
		if (SensorManager.getRotationMatrix(rmOrientationAccelMag, null,
				vAcceleration, vMagnetic))
		{
			SensorManager.getOrientation(rmOrientationAccelMag,
					vOrientationAccelMag);

			isOrientationValidAccelMag = true;
		}
	}

	/**
	 * Reinitialize the sensor and filter.
	 */
	public abstract void reset();

	protected abstract void onGyroscopeChanged();

	/**
	 * Initialize the mean filters.
	 */
	private void initFilters()
	{
		meanFilterAcceleration = new MeanFilterSmoothing();
		meanFilterAcceleration.setTimeConstant(meanFilterTimeConstant);

		meanFilterMagnetic = new MeanFilterSmoothing();
		meanFilterMagnetic.setTimeConstant(meanFilterTimeConstant);

		meanFilterGyroscope = new MeanFilterSmoothing();
		meanFilterGyroscope.setTimeConstant(meanFilterTimeConstant);
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{

	}
}
