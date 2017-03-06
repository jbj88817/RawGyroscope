package us.bojie.rawgyroscope.gyroscope;

import android.content.Context;
import android.hardware.SensorManager;

import com.kircherelectronics.gyroscopeexplorer.activity.filter.kalman.RotationKalmanFilter;
import com.kircherelectronics.gyroscopeexplorer.activity.filter.kalman.RotationMeasurementModel;
import com.kircherelectronics.gyroscopeexplorer.activity.filter.kalman.RotationProcessModel;

import org.apache.commons.math3.complex.Quaternion;

import java.util.Arrays;

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
 * ImuOKf stands for inertial movement unit orientation Kalman filter.
 * Quaternion is added because the filter applies the Kalman filter to the
 * quaternions from the gyroscope and acceleration/magnetic sensors,
 * respectively.
 * 
 * Kalman filtering, also known as linear quadratic estimation (LQE), is an
 * algorithm that uses a series of measurements observed over time, containing
 * noise (random variations) and other inaccuracies, and produces estimates of
 * unknown variables that tend to be more precise than those based on a single
 * measurement alone. More formally, the Kalman filter operates recursively on
 * streams of noisy input data to produce a statistically optimal estimate of
 * the underlying system state.
 * 
 * ImuOCKfQuaternion attempts to fuse magnetometer, gravity and gyroscope
 * sensors together to produce an accurate measurement of the rotation of the
 * device.
 * 
 * The magnetometer and acceleration sensors are used to determine one of the
 * two orientation estimations of the device. This measurement is subject to the
 * constraint that the device must not be accelerating and hard and soft-iron
 * distortions are not present in the local magnetic field..
 * 
 * The gyroscope is used to determine the second of two orientation estimations
 * of the device. The gyroscope can have a shorter response time and is not
 * effected by linear acceleration or magnetic field distortions, however it
 * experiences drift and has to be compensated periodically by the
 * acceleration/magnetic sensors to remain accurate.
 * 
 * Quaternions are used to integrate the measurements of the gyroscope and apply
 * the rotations to each sensors measurements via Kalman filter. This the ideal
 * method because quaternions are not subject to many of the singularties of
 * rotation matrices, such as gimbal lock.
 * 
 * The quaternion for the magnetic/acceleration sensor is only needed to apply
 * the weighted quaternion to the gyroscopes weighted quaternion via Kalman
 * filter to produce the fused rotation. No integrations are required.
 * 
 * The gyroscope provides the angular rotation speeds for all three axes. To
 * find the orientation of the device, the rotation speeds must be integrated
 * over time. This can be accomplished by multiplying the angular speeds by the
 * time intervals between sensor updates. The calculation produces the rotation
 * increment. Integrating these values again produces the absolute orientation
 * of the device. Small errors are produced at each iteration causing the gyro
 * to drift away from the true orientation.
 * 
 * To eliminate both the drift and noise from the orientation, the gyroscope
 * measurements are applied only for orientation changes in short time
 * intervals. The magnetometer/acceleration fusion is used for long time
 * intervals. This is equivalent to low-pass filtering of the accelerometer and
 * magnetic field sensor signals and high-pass filtering of the gyroscope
 * signals.
 * 
 * @author Kaleb
 *
 */
public class ImuOKfQuaternion extends Orientation
{
	// Developer Note: This is very much a work in progress. The filter works
	// for short periods of linear acceleration, and is stable under rotation
	// but has not be tested robustly.

	private boolean isInitialOrientationValid = false;

	// copy the new gyro values into the gyro array
	// convert the raw gyro data into a rotation vector
	private double[] vDeltaGyroscope = new double[4];
	private double[] qvOrientationAccelMag = new double[4];
	private double[] qvOrientationGyroscope = new double[4];
	private float[] qvFusedOrientation = new float[4];

	// rotation matrix from gyro data
	private float[] rmFusedOrientation = new float[9];

	// final orientation angles from sensor fusion
	private float[] vFusedOrientation = new float[3];

	private RotationKalmanFilter kalmanFilter;

	private RotationProcessModel pm;
	private RotationMeasurementModel mm;

	private Quaternion quatGyroDelta;
	private Quaternion quatGyro;
	private Quaternion quatAccelMag;

	public ImuOKfQuaternion(Context context)
	{
		super(context);

		pm = new RotationProcessModel();
		mm = new RotationMeasurementModel();

		kalmanFilter = new RotationKalmanFilter(pm, mm);
	}

	/**
	 * Get the orientation of the device. This method can be called *only* after
	 * setAcceleration(), setMagnetic() and getGyroscope() have been called.
	 * 
	 * @return float[] an array containing the linear acceleration of the device
	 *         where values[0]: azimuth, rotation around the Z axis. values[1]:
	 *         pitch, rotation around the X axis. values[2]: roll, rotation
	 *         around the Y axis. with respect to the Android coordinate system.
	 */
	public float[] getOrientation()
	{
		if (isOrientationValidAccelMag)
		{
			calculateFusedOrientation();
		}

		return vFusedOrientation;
	}

	@Override
	protected void onGyroscopeChanged()
	{
		// Don't start until accelerometer/magnetometer orientation has
		// been calculated. We need that initial orientation to base our
		// gyroscope rotation off of.
		if (!isOrientationValidAccelMag)
		{
			return;
		}

		// Only integrate when we can measure a delta time, so one iteration
		// must pass to initialize the timeStamp.
		if (this.timeStampGyroscopeOld != 0)
		{
			dT = (this.timeStampGyroscope - this.timeStampGyroscopeOld) * NS2S;

			getRotationVectorFromGyro();
		}

		// measurement done, save current time for next interval
		this.timeStampGyroscopeOld = this.timeStampGyroscope;
	}

	/**
	 * Reinitialize the sensor and filter.
	 */
	public void reset()
	{
		// copy the new gyro values into the gyro array
		// convert the raw gyro data into a rotation vector
		vDeltaGyroscope = new double[4];
		qvOrientationAccelMag = new double[4];
		qvOrientationGyroscope = new double[4];
		qvFusedOrientation = new float[4];

		// rotation matrix from gyro data
		rmFusedOrientation = new float[9];

		// final orientation angles from sensor fusion
		vFusedOrientation = new float[3];

		pm = new RotationProcessModel();
		mm = new RotationMeasurementModel();

		kalmanFilter = new RotationKalmanFilter(pm, mm);

		quatGyroDelta = null;
		quatGyro = null;
		quatAccelMag = null;

		isInitialOrientationValid = false;
		isOrientationValidAccelMag = false;
	}

	protected void calculateOrientationAccelMag()
	{
		super.calculateOrientationAccelMag();

		getRotationVectorFromAccelMag(vOrientationAccelMag);

		// Get an initial orientation vector from the acceleration and magnetic
		// sensors.
		if (isOrientationValidAccelMag && !isInitialOrientationValid)
		{
			quatGyro = new Quaternion(quatAccelMag.getScalarPart(),
					quatAccelMag.getVectorPart());

			isInitialOrientationValid = true;
		}
	}

	/**
	 * Create an angle-axis vector, in this case a unit quaternion, from the
	 * provided Euler angle's (presumably from SensorManager.getOrientation()).
	 * 
	 * Equation from
	 * http://www.euclideanspace.com/maths/geometry/rotations/conversions
	 * /eulerToQuaternion/
	 * 
	 * @param orientation
	 */
	private void getRotationVectorFromAccelMag(float[] orientation)
	{
		// Assuming the angles are in radians.

		// getOrientation() values:
		// values[0]: azimuth, rotation around the Z axis.
		// values[1]: pitch, rotation around the X axis.
		// values[2]: roll, rotation around the Y axis.

		// Heading, Azimuth, Yaw
		double c1 = Math.cos(-orientation[0] / 2);
		double s1 = Math.sin(-orientation[0] / 2);

		// Pitch, Attitude
		// The equation assumes the pitch is pointed in the opposite direction
		// of the orientation vector provided by Android, so we invert it.
		double c2 = Math.cos(-orientation[1] / 2);
		double s2 = Math.sin(-orientation[1] / 2);

		// Roll, Bank
		double c3 = Math.cos(orientation[2] / 2);
		double s3 = Math.sin(orientation[2] / 2);

		double c1c2 = c1 * c2;
		double s1s2 = s1 * s2;

		double w = c1c2 * c3 - s1s2 * s3;
		double x = c1c2 * s3 + s1s2 * c3;
		double y = s1 * c2 * c3 + c1 * s2 * s3;
		double z = c1 * s2 * c3 - s1 * c2 * s3;

		// The quaternion in the equation does not share the same coordinate
		// system as the Android gyroscope quaternion we are using. We reorder
		// it here.

		// Android X (pitch) = Equation Z (pitch)
		// Android Y (roll) = Equation X (roll)
		// Android Z (azimuth) = Equation Y (azimuth)

		qvOrientationAccelMag[0] = z;
		qvOrientationAccelMag[1] = x;
		qvOrientationAccelMag[2] = y;
		qvOrientationAccelMag[3] = w;

		quatAccelMag = new Quaternion(w, z, x, y);
	}

	/**
	 * Calculates a rotation vector from the gyroscope angular speed values.
	 * 
	 * @param gyroValues
	 * @param deltaRotationVector
	 * @param timeFactor
	 * @see http://developer.android
	 *      .com/reference/android/hardware/SensorEvent.html#values
	 */
	private void getRotationVectorFromGyro()
	{
		// Calculate the angular speed of the sample
		float magnitude = (float) Math.sqrt(Math.pow(vGyroscope[0], 2)
				+ Math.pow(vGyroscope[1], 2) + Math.pow(vGyroscope[2], 2));

		// Normalize the rotation vector if it's big enough to get the axis
		if (magnitude > EPSILON)
		{
			vGyroscope[0] /= magnitude;
			vGyroscope[1] /= magnitude;
			vGyroscope[2] /= magnitude;
		}

		// Integrate around this axis with the angular speed by the timestep
		// in order to get a delta rotation from this sample over the timestep
		// We will convert this axis-angle representation of the delta rotation
		// into a quaternion before turning it into the rotation matrix.
		float thetaOverTwo = magnitude * dT / 2.0f;
		float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
		float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

		vDeltaGyroscope[0] = sinThetaOverTwo * vGyroscope[0];
		vDeltaGyroscope[1] = sinThetaOverTwo * vGyroscope[1];
		vDeltaGyroscope[2] = sinThetaOverTwo * vGyroscope[2];
		vDeltaGyroscope[3] = cosThetaOverTwo;

		// Create a new quaternion object base on the latest rotation
		// measurements...
		quatGyroDelta = new Quaternion(vDeltaGyroscope[3], Arrays.copyOfRange(
				vDeltaGyroscope, 0, 3));

		// Since it is a unit quaternion, we can just multiply the old rotation
		// by the new rotation delta to integrate the rotation.
		quatGyro = quatGyro.multiply(quatGyroDelta);
	}

	/**
	 * Calculate the fused orientation.
	 */
	private void calculateFusedOrientation()
	{
		qvOrientationGyroscope[0] = (float) quatGyro.getVectorPart()[0];
		qvOrientationGyroscope[1] = (float) quatGyro.getVectorPart()[1];
		qvOrientationGyroscope[2] = (float) quatGyro.getVectorPart()[2];
		qvOrientationGyroscope[3] = (float) quatGyro.getScalarPart();

		// Apply the Kalman filter... Note that the prediction and correction
		// inputs could be swapped, but the filter is much more stable in this
		// configuration.
		kalmanFilter.predict(qvOrientationGyroscope);
		kalmanFilter.correct(qvOrientationAccelMag);

		// Apply the new gyroscope delta rotation to the new Kalman filter
		// rotation estimation.
		quatGyro = new Quaternion(kalmanFilter.getStateEstimation()[3],
				Arrays.copyOfRange(kalmanFilter.getStateEstimation(), 0, 3));

		// Now we get a structure we can pass to get a rotation matrix, and then
		// an orientation vector from Android.
		qvFusedOrientation[0] = (float) kalmanFilter.getStateEstimation()[0];
		qvFusedOrientation[1] = (float) kalmanFilter.getStateEstimation()[1];
		qvFusedOrientation[2] = (float) kalmanFilter.getStateEstimation()[2];
		qvFusedOrientation[3] = (float) kalmanFilter.getStateEstimation()[3];

		// We need a rotation matrix so we can get the orientation vector...
		// Getting Euler
		// angles from a quaternion is not trivial, so this is the easiest way,
		// but perhaps
		// not the fastest way of doing this.
		SensorManager.getRotationMatrixFromVector(rmFusedOrientation,
				qvFusedOrientation);

		// Get the fused orienatation
		SensorManager.getOrientation(rmFusedOrientation, vFusedOrientation);
	}

	@Override
	public void setFilterCoefficient(float filterCoefficient)
	{

	}
}
