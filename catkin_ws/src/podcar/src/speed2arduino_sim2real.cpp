#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <iostream>
using namespace std;


// Author: Fanta Camara, July 2019, with new edits in Dec. 2021 and Jan. 2022

//ROS node for the Pololu PID controller
//represents only the Pololu
//does not care what it is being used for.
//Input: int 0-4095 to set desired PID position
//Output: int 0-4095 containign actual current position

//assumes something like the following in udev rules /etc/udev/rules.d/99-usb-serial.rules :
//SUBSYSTEM=="tty", ATTRS{idVendor} =="1ffb", ENV{ID_USB_INTERFACE_NUM}=="00"  SYMLINK+="ttyPololuCOM"
//SUBSYSTEM=="tty", ATTRS{idVendor} =="1ffb", ENV{ID_USB_INTERFACE_NUM}=="02"  SYMLINK+="ttyPololuTTL"
//NB the pololu has two virtual serial ports on a single USB interface.

//from: https://www.pololu.com/docs/0J38/4.i.1
// Uses POSIX functions to send and receive data from a jrk.
// NOTE: The jrk's input mode must be "Serial".
// NOTE: The jrk's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <sys/ioctl.h>

#include <errno.h>
#include <string.h>
//#ifdef _WIN32
//#define O_NOCTTY 0
//#else

#define MIN_VEL_FORWARD 0.65
#define MIN_VEL_BACKWARD -0.65

#define MAX_VEL_FORWARD 1
#define MAX_VEL_BACKWARD -1

int fd;  //file descriptor for serial port
ros::Publisher pub;  //publisher

bool received_reponse_to_last_command = true;
char buffer[100];
int buffer_idx_next = 0;

int set_interface_attribs (int fd, int speed)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        //tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

	tcflush(fd, TCIFLUSH);
        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}


float rescale(float x, float oldmin, float oldmax, float newmin, float newmax)
{
	float r = (x-oldmin)/(oldmax-oldmin);
	float out = newmin + r*(newmax-newmin);
	return out;
}

int write2port(char *cmd)
{
	if (write(fd, cmd, sizeof(cmd)) == -1)    // write received cmd to port
 	{
		perror("error writing");
		return -1;
	}
	return 0;
}

int read2port(char * chars, int nwaiting)
{

	if(read(fd, chars, nwaiting) < 0)  // read response from Arduino
  	{
		perror("error reading");
		return -1;		
		
  	}
	
	return 0;

}


void callback_cmd(const std_msgs::Float64::ConstPtr& msg)
{ 

		printf("\nCharles is here\n");
		float velocity_ms = msg->data;
		float velocity;
		// HACK probably best to convert direct from ms to arduino bytes later on - need to actually measure speeds per byte though	
		// convert to -1:1 range  (careful, these values are also used in joystick2speedms)
		if(velocity_ms>=0.01 && velocity_ms < 0.4)
			velocity = MIN_VEL_FORWARD + velocity_ms; //velocity_ms/3.0;
		else if (velocity_ms >= MIN_VEL_FORWARD && velocity_ms < MAX_VEL_FORWARD)
			velocity = velocity_ms;
		else if (velocity_ms >= 0.4)
			velocity = MAX_VEL_FORWARD;
		else if (velocity_ms <= -0.01 && velocity_ms > -0.4)
			velocity = MIN_VEL_BACKWARD + velocity_ms;//velocity_ms/1.0;  //reverse is slower
		else if (velocity_ms <= MIN_VEL_BACKWARD && velocity_ms > MAX_VEL_BACKWARD)
			velocity = velocity_ms;
		else if (velocity_ms <= -0.4)
			velocity = MAX_VEL_BACKWARD;
		else if (velocity_ms > -0.01 &&  velocity_ms < 0.01)
			velocity = 0.;
			
		printf("Velocity: %f\n", velocity);
		// speedbytes are converted linearly to voltage as 
		// V = (4096/V_usb)*speedbyte   : range 0:5V
		// where V_usb is the voltage supplied to arduino power. in theory 5V but often 4.97V etc.
		// voltage V is then sent to the vehicle controller, which has 0V=fast reverse, 5V fast fwd
		// there is also a dead zone around 1.92V-2.71V used for STOP and needed at ignition-on. 
		// dead zone appears as 133-200 speedbytes for toughbook (different for other USB supplies)

		int byte_dead_zone_top = 220; // 200
		int byte_max_speed = 255;  // max accepted by arduino is 255 but can limit for safety here
		int byte_dead_zone_base = 140; // 132
		int byte_max_reverse = 103; //103   #max reverse would be 0 but we cap for safety here
		int byte_stop =  170; //180  //164
		

		// only write to serial if we are not still waiting for a confirm read
		if(received_reponse_to_last_command == true)
		{
			int speed_byte;
			
			if(abs(velocity)<0.01)
				speed_byte=byte_stop;  // dead range center, good to startup
				
			else if (velocity>=0.01)    // 240 is racing fast ; use less for safety for now eg 210
				speed_byte = int(byte_dead_zone_top+velocity*(byte_max_speed-byte_dead_zone_top));  // from top of dead range tocapped max
				
			else
				speed_byte = int(byte_dead_zone_base+velocity*(byte_dead_zone_base-byte_max_reverse));  // from bottom of dead range to cap max reverse
			
			char lineout[100];
			snprintf(lineout, sizeof(lineout), "FA:%i\r\n", speed_byte);   //"FA:%i\r\n"%speed_byte;   //140 is stop.  240 is fast fwd.  80 for fast reverse.

			//printf("lineout: %s", lineout);
			//char cmd[100];  
			//strcpy(cmd, lineout);  // convert string into list of char
			
			printf("New command: %s\n", lineout);
			write2port(lineout);
			printf("Cmd written: %s", lineout);
			
			received_reponse_to_last_command = false;    // cmd executed so time to get response from Arduino

		}

		int nwaiting;
		ioctl(fd, FIONREAD, &nwaiting);  // get the number of bytes waiting to be read
		printf("Bytes to be read: %i\n", nwaiting);

		if(nwaiting > 0)
		{
			char chars [nwaiting];		

			printf("start read\n");		
			int error = read2port(chars,  nwaiting);		

			printf("done read, error = %i\n", error);
			printf("Bytes read from Arduino: %s\n", chars);
		
			if(strlen(chars) != nwaiting)
				printf("read length diff from req n chars");  // printed in the terminal all the time, might become a problem in the future 
	
			for(int i = 0; i < strlen(chars); i++)
			{
				char c_chars = chars[i];
				buffer[buffer_idx_next] = c_chars;
				buffer_idx_next += 1;

				if(c_chars=='\n')
				{
					// finished a line
					//string linein = buffer.substr(0, buffer_idx_next - 1);  // extract the response from the buffer
					 
					for(int i = 0; i< 100; i++)
						buffer[i] = 'X';
					
					buffer_idx_next=0;
					received_reponse_to_last_command = true;   // enables us to send a new command
					
					// publish velocity for odometry
		      std_msgs::Float64 msg_out;
		      msg_out.data = velocity;
		      pub.publish(msg_out);
				}
			}
		}
     
	printf("\nChris is here\n");
}

int main(int argc, char **argv)
{
  cout << "Hello" << endl;
  ros::init(argc, argv, "speed2arduino");
  ros::NodeHandle n;
  
  pub = n.advertise<std_msgs::Float64>("speed4arduino", 10);

  printf("start\n");
  const char * device = "/dev/ttyArduino";  // Linux
  fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    printf("could not open Arduino\n");
    perror(device);
    return 1;
  }

  set_interface_attribs (fd, B9600);  // set speed to 115,200 bps
  printf("opened Arduino port\n");

 sleep(5);
  
 int nwaiting;
 ioctl(fd, FIONREAD, &nwaiting);  // get the number of bytes waiting to be read

 char line [nwaiting +1];
 printf("start read %i\n", nwaiting);		
 int error = read2port(line,  nwaiting);	
  
  printf("Msg from Arduino: %s\n", line);

  /*if(strcmp(line, "Car Throttle\r\n")!=0)
  {
     perror("Wrong port");
     return 0;
  }*/


  for(int i = 0; i <100; i++)    // init buffer
     buffer[i] = 'X';

  ros::Subscriber sub = n.subscribe("speedcmd_meterssec", 1, callback_cmd);
  printf("After return\n");
  ros::spin();
  
  close(fd);
  return 0;
}	
