/*
 * Complementary_filter.h
 *
 * Created: 2/13/2015 2:31:09 AM
 *  Author: Niraj
 */ 


#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_

class Complementary
{
    private: 
	        float a;	
			float angle;           //Angle calculated from first order complementary filter
			float angle2C;        //Angle calculated from second order complementary filter
			float k;
    public: 
	       Complementary()
		   {
			 a=0.02;         //Initially assumed values for tuning parameter of first order and second order complimentary filter
			 k=10;
			 angle=0;        //Initially assumed starting angles for first order and second order filter
			 angle2C=0; 
		   }


           float getangle(float newAngle, float newRate,float dt)     //First order complimentary filter
           {
	         angle= a* (angle + newRate * dt) + (1-a) * (newAngle);
	         return angle;
           }

           float get_secondorder_angle(float newAngle, float newRate,float dt)   //Second order complimentary filter
           {
	         float x1,y1,x2;
	         x1 = (newAngle - angle2C)*k*k;
	         y1 = dt*x1 + y1;
	         x2 = y1 + (newAngle -   angle2C)*2*k + newRate;
	         angle2C = dt*x2 + angle2C;

	         return angle2C;
           }
           void set_firstorder_tune(float tune)  //Used to set first order tuning parameter a
           {
	         a=tune;
	       }
           void set_secondorder_tune(float tune) //Used to set second order tuning parameter k
           {
	         k=tune;	
	       }
           void setAngle(float newangle)        // Used to set angle i.e, starting angle for fist order
           {
	         angle=newangle;	
	       }
           void setAngle2C(float newangle)      // Used to set angle i.e, starting angle for second order
           {
	         angle2C=newangle;	
	       }	
};



#endif /* COMPLEMENTARY_FILTER_H_ */