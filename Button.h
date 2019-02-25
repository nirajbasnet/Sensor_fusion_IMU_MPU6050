/*
 * Button.h
 *
 * Created: 2/15/2015 2:42:07 AM
 *  Author: Niraj
 */


#ifndef BUTTON_H_
#define BUTTON_H_

#include <avr/io.h>
#include <util/delay.h>
#define PULL_UP 0
#define PULL_DOWN 1

class Button
{
	private:bool state;
	        char pullstate;
			unsigned char port;
			unsigned char pin_number;
			float *variable;

	public:
	       Button(unsigned char pullpush)
		   {
			   state=false;
			   pullstate=pullpush;
		   }
		   Button(unsigned char p,unsigned char pin,unsigned char pullpush)
		   {
			   state=false;
			   pullstate=pullpush;
			   port=p;
			   pin_number=pin;

		   }
		   bool buttonpressed()
		   {
             if(pullstate)
			   {
				   if(bit_is_set(port,pin_number))
                   {
                       _delay_ms(10);
                        if(bit_is_set(port,pin_number))
                            return true;
				            else
				            return false;
                   }

			   }
			   else
			   {
				   if(bit_is_clear(port,pin_number))
                   {
                       _delay_ms(10);
                       if(bit_is_clear(port,pin_number))
                       return true;
			           else
			           return false;
                   }
                }
		   }
		   void setPin_number(unsigned char p,unsigned char pin)
		   {
			 port=p;
			 pin_number=pin;
		   }
		   void set_InternalPullup()
		   {

		   }
		   void set_Increment(float *value,float increment_step)
		   {
             *value=*value+increment_step;
		   }
		   void set_Decrement(float *value,float decrement_step)
           {
             *value=*value-decrement_step;
           }

};



#endif /* BUTTON_H_ */
