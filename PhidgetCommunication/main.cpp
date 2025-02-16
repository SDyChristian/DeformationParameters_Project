#include <stdio.h>
#include <pthread.h>
#include <iostream>
#include <time.h>
#include <phidget21.h>

#include "keyboard_hit.h"
#include "sleep_time.h"

// routines
void *interface( void *ptr ); // thread
void *analogue( void *ptr ); // thread

// global variables, arrays, pointers, structures, etc.
bool STOP = false; // flags
double time_ = 0.0, output = 0.0;
const double dt = 0.01;
double i=0, j=4;

// main program
int main(void)
{
  pthread_t thread_interface, thread_analogue; // threads' pointer
  
	// initialization
  pthread_create( &thread_interface, NULL, interface, NULL);
  pthread_create( &thread_analogue, NULL, analogue, NULL);

	// program termination 
  pthread_join( thread_interface, NULL );
  pthread_join( thread_analogue, NULL );
	printf("Program End\n");

	return(0);
}

// interface thread: Thread responsible for managing keyboard input.
void *interface( void *ptr ) {
				int pressed_key = 0; // to catch the pressed key

				// initialization
				init_keyboard(); // start keyboard reading

				// loop 
				while(!STOP) {
								// display
								printf("Time = %g, i = %g, j = %g\n", time_, output,j);
								
								//  if a key is pressed
								if(kbhit()) {
												pressed_key = readch(); // get pressed key
												if (pressed_key == 27) STOP = true; // esc
								}

								// wait
								sleep_time(dt);
				}

				// termination
				close_keyboard(); // stop reading the keyboard
				pthread_exit(NULL); // terminate thread
}

// analogue reading: Thread responsible for interacting with the Phidget analog input/output device.
void *analogue( void *ptr ) {
				CPhidgetAnalogHandle analog = 0; // declare an Analog handle

				// initialization
				CPhidgetAnalog_create(&analog); // create the Analog object
				CPhidget_open((CPhidgetHandle)analog, -1); // open the device
				CPhidget_waitForAttachment((CPhidgetHandle)analog, 10000); // for device

				// loop
				while (!STOP) {
				  
								// signal
							
								if(i>13.5){
								  i=0;
								  j=j+.2;
								  if(j>5)
								    j=1;
								}
								output = i;
								
								// set output voltage
																
								if(i<=6){
								  CPhidgetAnalog_setVoltage(analog, 0, output);
								  CPhidgetAnalog_setEnabled(analog, 0, PTRUE);
								  CPhidgetAnalog_setVoltage(analog, 1, output);
								  CPhidgetAnalog_setEnabled(analog, 1, PTRUE);
								  CPhidgetAnalog_setVoltage(analog, 2, output);
								  CPhidgetAnalog_setEnabled(analog, 2, PTRUE);
								}
								else{
									// Offset done due to robot imperfections
								  CPhidgetAnalog_setVoltage(analog, 0, j);
								  CPhidgetAnalog_setEnabled(analog, 0, PTRUE);
								  CPhidgetAnalog_setVoltage(analog, 1, output-6);
								  CPhidgetAnalog_setEnabled(analog, 1, PTRUE);
								  CPhidgetAnalog_setVoltage(analog, 2, j+.05);
								  CPhidgetAnalog_setEnabled(analog, 2, PTRUE);
								  }

								// wait
								if(i==0)
								sleep_time(8);
								else
								sleep_time(2);
								time_=time_+dt;
								sleep_time(dt); 
								i = i + 0.1;	
				}

				// termination
				CPhidgetAnalog_setEnabled(analog, 0, 0.0); // set 0V
				CPhidgetAnalog_setEnabled(analog, 1, 0.0); // set 0V
				CPhidgetAnalog_setEnabled(analog, 2, 0.0); // set 0V
				CPhidget_close((CPhidgetHandle)analog);
				CPhidget_delete((CPhidgetHandle)analog);
				pthread_exit(NULL); // terminate thread
}
