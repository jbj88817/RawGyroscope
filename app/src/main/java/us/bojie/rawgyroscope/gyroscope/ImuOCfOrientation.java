package us.bojie.rawgyroscope.gyroscope;

import android.content.Context;
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
 * ImuOCf stands for inertial movement unit orientation complementary filter.
 * Orientation is added because the filter applies the complementary filter to
 * the Euler angles (orientation) from the gyroscope and acceleration/magnetic
 * sensors, respectively.
 * 
 * The complementary filter is a frequency domain filter. In its strictest
 * sense, the definition of a complementary filter refers to the use of two or
 * more transfer functions, which are mathematical complements of one another.
 * Thus, if the data from one sensor is operated on by G(s), then the data from
 * the other sensor is operated on by I-G(s), and the sum of the transfer
 * functions is I, the identity matrix.
 * 
 * ImuOCfOrientation attempts to fuse magnetometer, gravity and gyroscope
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
 * Euler angles and rotation matrices are used to integrate the measurements of
 * the gyroscope and to convert the orientation vector back into a rotation
 * matrix so it can be integrated by the gyroscope. This is not ideal because
 * Euler angles suffer from singularities known as gimbal lock.
 * 
 * The rotation matrix for the magnetic/acceleration sensor is only required to
 * determine the orientation vector, no integrations are required.
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
 * @version %I%, %G%
 * @see http 
 *      ://developer.android.com/reference/android/hardware/SensorEvent.html#
 *      values
 * 
 *      The gist of this algorithm was written by Paul Lawitzki.
 * @see http://www.thousand-thoughts.com/2012/03/android-sensor-fusion-tutorial/
 * 
 */

public class ImuOCfOrientation extends Orientation
{
	private static final String tag = ImuOCfOrientation.class.getSimpleName();

	private boolean isInitialOrientationValid = false;

	// The coefficient for the filter... 0.5 = means it is averaging the two
	// transfer functions (rotations from the gyroscope and
	// acceleration/magnetic, respectively).
	public float filterCoefficient = 0.5f;

	private float omegaMagnitude = 0;

	private float thetaOverTwo = 0;
	private float sinThetaOverTwo = 0;
	private float cosThetaOverTwo = 0;

	// rotation matrix from gyro data
	private float[] rmGyroscope = new float[9];

	// orientation angles from gyro matrix
	private float[] vOrientationGyroscope = new float[3];

	// final orientation angles from sensor fusion
	private float[] vOrientationFused = new float[3];

	// copy the new gyro values into the gyro array
	// convert the raw gyro data into a rotation vector
	private float[] vDeltaGyroscope = new float[4];

	// convert rotation vector into rotation matrix
	private float[] rmDeltaGyroscope = new float[9];

	/**
	 * Initialize a singleton instance.
	 * 
	 * @param gravitySubject
	 *            the gravity subject.
	 * @param gyroscopeSubject
	 *            the gyroscope subject.
	 * @param magneticSubject
	 *            the magnetic subject.
	 */
	public ImuOCfOrientation(Context context)
	{
		super(context);

		// The orientation vector for the gyroscope.
		vOrientationGyroscope[0] = 0.0f;
		vOrientationGyroscope[1] = 0.0f;
		vOrientationGyroscope[2] = 0.0f;

		// Initialize gyroMatrix with identity matrix. This is important because
		// we will need to initialize this matrix... either to the orientation
		// of the device relative to earth frame or to the initial local frame.
		rmGyroscope[0] = 1.0f;
		rmGyroscope[1] = 0.0f;
		rmGyroscope[2] = 0.0f;
		rmGyroscope[3] = 0.0f;
		rmGyroscope[4] = 1.0f;
		rmGyroscope[5] = 0.0f;
		rmGyroscope[6] = 0.0f;
		rmGyroscope[7] = 0.0f;
		rmGyroscope[8] = 1.0f;

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

		// Fuse the gyroscope and acceleration/magnetic sensor orientations
		// together via complementary filter to produce a new, fused
		// orientation.
		calculateFusedOrientation();

		return vOrientationFused;
	}

	/**
	 * The complementary filter coefficient, a floating point value between 0-1,
	 * exclusive of 0, inclusive of 1.
	 * 
	 * @param filterCoefficient
	 */
	public void setFilterCoefficient(float filterCoefficient)
	{
		this.filterCoefficient = filterCoefficient;
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
		omegaMagnitude = 0;

		thetaOverTwo = 0;
		sinThetaOverTwo = 0;
		cosThetaOverTwo = 0;

		// rotation matrix from gyro data
		rmGyroscope = new float[9];

		// orientation angles from gyro matrix
		vOrientationGyroscope = new float[3];

		// final orientation angles from sensor fusion
		vOrientationFused = new float[3];

		// copy the new gyro values into the gyro array
		// convert the raw gyro data into a rotation vector
		vDeltaGyroscope = new float[4];

		// convert rotation vector into rotation matrix
		rmDeltaGyroscope = new float[9];

		isOrientationValidAccelMag = false;
	}

	protected void calculateOrientationAccelMag()
	{
		super.calculateOrientationAccelMag();

		// Get an initial orientation vector from the acceleration and magnetic
		// sensors.
		if (isOrientationValidAccelMag && !isInitialOrientationValid)
		{
			rmGyroscope = matrixMultiplication(rmGyroscope,
					rmOrientationAccelMag);

			isInitialOrientationValid = true;
		}
	}

	/**
	 * Calculate the fused orientation. The gist of this algorithm was written
	 * by Paul Lawitzki.
	 * 
	 * @see http 
	 *      ://www.thousand-thoughts.com/2012/03/android-sensor-fusion-tutorial/
	 */
	private void calculateFusedOrientation()
	{
		float oneMinusCoeff = (1.0f - filterCoefficient);

		/*
		 * Fix for 179� <--> -179� transition problem: Check whether one of the
		 * two orientation angles (gyro or accMag) is negative while the other
		 * one is positive. If so, add 360� (2 * math.PI) to the negative value,
		 * perform the sensor fusion, and remove the 360� from the result if it
		 * is greater than 180�. This stabilizes the output in
		 * positive-to-negative-transition cases.
		 */

		// azimuth
		if (vOrientationGyroscope[0] < -0.5 * Math.PI
				&& vOrientationAccelMag[0] > 0.0)
		{
			vOrientationFused[0] = (float) (filterCoefficient
					* (vOrientationGyroscope[0] + 2.0 * Math.PI) + oneMinusCoeff
					* vOrientationAccelMag[0]);
			vOrientationFused[0] -= (vOrientationFused[0] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else if (vOrientationAccelMag[0] < -0.5 * Math.PI
				&& vOrientationGyroscope[0] > 0.0)
		{
			vOrientationFused[0] = (float) (filterCoefficient
					* vOrientationGyroscope[0] + oneMinusCoeff
					* (vOrientationAccelMag[0] + 2.0 * Math.PI));
			vOrientationFused[0] -= (vOrientationFused[0] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else
		{
			vOrientationFused[0] = filterCoefficient * vOrientationGyroscope[0]
					+ oneMinusCoeff * vOrientationAccelMag[0];
		}

		// pitch
		if (vOrientationGyroscope[1] < -0.5 * Math.PI
				&& vOrientationAccelMag[1] > 0.0)
		{
			vOrientationFused[1] = (float) (filterCoefficient
					* (vOrientationGyroscope[1] + 2.0 * Math.PI) + oneMinusCoeff
					* vOrientationAccelMag[1]);
			vOrientationFused[1] -= (vOrientationFused[1] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else if (vOrientationAccelMag[1] < -0.5 * Math.PI
				&& vOrientationGyroscope[1] > 0.0)
		{
			vOrientationFused[1] = (float) (filterCoefficient
					* vOrientationGyroscope[1] + oneMinusCoeff
					* (vOrientationAccelMag[1] + 2.0 * Math.PI));
			vOrientationFused[1] -= (vOrientationFused[1] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else
		{
			vOrientationFused[1] = filterCoefficient * vOrientationGyroscope[1]
					+ oneMinusCoeff * vOrientationAccelMag[1];
		}

		// roll
		if (vOrientationGyroscope[2] < -0.5 * Math.PI
				&& vOrientationAccelMag[2] > 0.0)
		{
			vOrientationFused[2] = (float) (filterCoefficient
					* (vOrientationGyroscope[2] + 2.0 * Math.PI) + oneMinusCoeff
					* vOrientationAccelMag[2]);
			vOrientationFused[2] -= (vOrientationFused[2] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else if (vOrientationAccelMag[2] < -0.5 * Math.PI
				&& vOrientationGyroscope[2] > 0.0)
		{
			vOrientationFused[2] = (float) (filterCoefficient
					* vOrientationGyroscope[2] + oneMinusCoeff
					* (vOrientationAccelMag[2] + 2.0 * Math.PI));
			vOrientationFused[2] -= (vOrientationFused[2] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else
		{
			vOrientationFused[2] = filterCoefficient * vOrientationGyroscope[2]
					+ oneMinusCoeff * vOrientationAccelMag[2];
		}

		// overwrite gyro matrix and orientation with fused orientation
		// to comensate gyro drift
		rmGyroscope = getRotationMatrixFromOrientation(vOrientationFused);

		System.arraycopy(vOrientationFused, 0, vOrientationGyroscope, 0, 3);
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
		// This code is taken from the Android samples/developer reference. It
		// creates a unit quaternion which is then transformed into a rotation
		// matrix before it is integrated. This is not ideal, but it works.

		// Calculate the angular speed of the sample
		omegaMagnitude = (float) Math.sqrt(Math.pow(vGyroscope[0], 2)
				+ Math.pow(vGyroscope[1], 2) + Math.pow(vGyroscope[2], 2));

		// Normalize the rotation vector if it's big enough to get the axis
		if (omegaMagnitude > EPSILON)
		{
			vGyroscope[0] /= omegaMagnitude;
			vGyroscope[1] /= omegaMagnitude;
			vGyroscope[2] /= omegaMagnitude;
		}

		// Integrate around this axis with the angular speed by the timestep
		// in order to get a delta rotation from this sample over the timestep
		// We will convert this axis-angle representation of the delta rotation
		// into a quaternion before turning it into the rotation matrix.
		thetaOverTwo = omegaMagnitude * dT / 2.0f;
		sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
		cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

		vDeltaGyroscope[0] = sinThetaOverTwo * vGyroscope[0];
		vDeltaGyroscope[1] = sinThetaOverTwo * vGyroscope[1];
		vDeltaGyroscope[2] = sinThetaOverTwo * vGyroscope[2];
		vDeltaGyroscope[3] = cosThetaOverTwo;

		// Get the rotation matrix from the gyroscope
		SensorManager.getRotationMatrixFromVector(rmDeltaGyroscope,
				vDeltaGyroscope);

		// Apply the new rotation interval on the gyroscope based rotation
		// matrix to form a composite rotation matrix. The product of two
		// rotation matricies is a rotation matrix...
		// Multiplication of rotation matrices corresponds to composition of
		// rotations... Which in this case are the rotation matrix from the
		// fused orientation and the rotation matrix from the current
		// gyroscope
		// outputs.
		rmGyroscope = matrixMultiplication(rmGyroscope, rmDeltaGyroscope);

		// Get the gyroscope based orientation from the composite rotation
		// matrix. This orientation will be fused via complementary filter
		// with
		// the orientation from the acceleration sensor and magnetic sensor.
		SensorManager.getOrientation(rmGyroscope, vOrientationGyroscope);
	}

	/**
	 * Get the rotation matrix from the current orientation. Android Sensor
	 * Manager does not provide a method to transform the orientation into a
	 * rotation matrix, only the orientation from a rotation matrix. The basic
	 * rotations can be found in Wikipedia with the caveat that the rotations
	 * are *transposed* relative to what is required for this method.
	 * 
	 * @param The
	 *            device orientation.
	 * @return The rotation matrix from the orientation.
	 * 
	 * @see http://en.wikipedia.org/wiki/Rotation_matrix
	 */
	private float[] getRotationMatrixFromOrientation(float[] orientation)
	{
		float[] xM = new float[9];
		float[] yM = new float[9];
		float[] zM = new float[9];

		float sinX = (float) Math.sin(orientation[1]);
		float cosX = (float) Math.cos(orientation[1]);
		float sinY = (float) Math.sin(orientation[2]);
		float cosY = (float) Math.cos(orientation[2]);
		float sinZ = (float) Math.sin(orientation[0]);
		float cosZ = (float) Math.cos(orientation[0]);

		// rotation about x-axis (pitch)
		xM[0] = 1.0f;
		xM[1] = 0.0f;
		xM[2] = 0.0f;
		xM[3] = 0.0f;
		xM[4] = cosX;
		xM[5] = sinX;
		xM[6] = 0.0f;
		xM[7] = -sinX;
		xM[8] = cosX;

		// rotation about y-axis (roll)
		yM[0] = cosY;
		yM[1] = 0.0f;
		yM[2] = sinY;
		yM[3] = 0.0f;
		yM[4] = 1.0f;
		yM[5] = 0.0f;
		yM[6] = -sinY;
		yM[7] = 0.0f;
		yM[8] = cosY;

		// rotation about z-axis (azimuth)
		zM[0] = cosZ;
		zM[1] = sinZ;
		zM[2] = 0.0f;
		zM[3] = -sinZ;
		zM[4] = cosZ;
		zM[5] = 0.0f;
		zM[6] = 0.0f;
		zM[7] = 0.0f;
		zM[8] = 1.0f;

		// Build the composite rotation... rotation order is y, x, z (roll,
		// pitch, azimuth)
		float[] resultMatrix = matrixMultiplication(xM, yM);
		resultMatrix = matrixMultiplication(zM, resultMatrix);
		return resultMatrix;
	}

	/**
	 * Multiply A by B.
	 * 
	 * @param A
	 * @param B
	 * @return A*B
	 */
	private float[] matrixMultiplication(float[] A, float[] B)
	{
		float[] result = new float[9];

		result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
		result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
		result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

		result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
		result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
		result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

		result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
		result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
		result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

		return result;
	}

}
