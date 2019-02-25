/*
 * Kalman_filter.h
 *
 * Created: 2/13/2015 2:41:03 AM
 *  Author: Niraj
 */ 


#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

class Kalman
{
	private:
	       double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	       double bias;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	       double rate;  // Unbiased rate calculated from the rate and the calculated bias 
		   
		   double Q_angle;   // Process noise variance for the accelerometer
		   double Q_bias;    // Process noise variance for the gyro bias
		   double R_measure; // Measurement noise variance 

	       double P[2][2]; // Error covariance 2x2 matrix
	       double K[2];    // Kalman gain -2x1 vector
	       double y;       // Angle difference
	       double S;       // Estimate error

	public:
	      Kalman()
	      {
		  Q_angle = 0.001;    //Initially assumed values for noise parameters
		  Q_bias = 0.003;
		  R_measure = 0.03;

		  angle = 0; // Reset the angle to 0
		  bias = 0;  // Reset bias to 0

		  P[0][0] = 0; // since the bias is 0 and we know the starting angle, the error covariance matrix is set like this
		  P[0][1] = 0;
		  P[1][0] = 0;
		  P[1][1] = 0;
	      }
	// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
	      double getKalmanAngle(double newAngle, double newRate, double dt)
	      {

		 // Predict stage-Project the state ahead

	      /* Step 1 */
		  rate = newRate - bias;
		  angle += dt * rate;

		 // Update estimation error covariance - Project the error covariance ahead
		 /* Step 2 */
		 P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
		 P[0][1] -= dt * P[1][1];
		 P[1][0] -= dt * P[1][1];
		 P[1][1] += Q_bias * dt;

		 //  Measurement Update- Update estimate with measurement zk (newAngle)
		 /* Step 3 */
		 y = newAngle - angle;

		 // Calculate Kalman gain - Compute the Kalman gain
		 /* Step 4 */
		 S = P[0][0] + R_measure;

		 /* Step 5 */
		 K[0] = P[0][0] / S;
		 K[1] = P[1][0] / S;

		/* Step 6 */
		 angle += K[0] * y;
		 bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		P[0][0] -= K[0] * P[0][0];
		P[0][1] -= K[0] * P[0][1];
		P[1][0] -= K[1] * P[0][0];
		P[1][1] -= K[1] * P[0][1];

		return angle;
        }
	void setAngle(double newAngle)  // Used to set angle, this should be set as the starting angle
	{ 
		 angle = newAngle; 
    }
	 
	/* Parameters to tune the Kalman filter */
	void setQangle(double newQ_angle) 
	{ 
		Q_angle = newQ_angle;
    }
	void setQbias(double newQ_bias)
    {
		Q_bias = newQ_bias; 
    }
	void setRmeasure(double newR_measure)
	{
	    R_measure = newR_measure; 
	}

};



#endif /* KALMAN_FILTER_H_ */