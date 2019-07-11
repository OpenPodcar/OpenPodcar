#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <iostream>
using namespace std;


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
//#endif

#include <errno.h>
#include <string.h>
//#ifdef _WIN32
//#define O_NOCTTY 0
//#else


int fd;  //file descriptor for serial port
ros::Publisher pub;  //publisher 


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

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}


// Reads a variable from the jrk.
// The 'command' argument must be one of the two-byte variable-reading
// commands documented in the "Variable Reading Commands" section of
// the jrk user's guide.
int jrkGetVariable(int fd, unsigned char command)
{
  if(write(fd, &command, 1) == -1)
  {
    perror("error writing");
    return -1;
  }

  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }

  return response[0] + 256*response[1];
}

// Gets the value of the jrk's Feedback variable (0-4095).
int jrkGetFeedback(int fd)
{
  return jrkGetVariable(fd, 0xA5);
}

// Gets the value of the jrk's Target variable (0-4095).
int jrkGetTarget(int fd)
{
  return jrkGetVariable(fd, 0xA3);
}

// Sets the jrk's Target variable (0-4095).
int jrkSetTarget(int fd, unsigned short target)
{
  unsigned char command[] = {0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
}

void pidCallback(const std_msgs::Int64::ConstPtr& msg)
{
  int target = msg->data;
  printf("msgcallback %i\n", target);

  int feedback = jrkGetFeedback(fd);
  printf("Current Feedback is %d.\n", feedback);

  int curTarget = jrkGetTarget(fd);
  printf("Current Target is %d.\n", curTarget);

// int newTarget = 1000;  //far left
// int newTarget = 2000;  //center OK
// int newTarget = 2500;  //far right
  printf("Setting Target to %d.\n", target);
  jrkSetTarget(fd, target);


  std_msgs::Int64 msg_out;
  msg_out.data = feedback;
  pub.publish(msg_out);

}


int main(int argc, char **argv)
{
  cout << "Hello" << endl;
  ros::init(argc, argv, "pololuController");
  ros::NodeHandle n;

  pub = n.advertise<std_msgs::Int64>("pololuFdbk", 1000);


  printf("start\n");
  const char * device = "/dev/ttyPololuCOM";  // Linux
  fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    printf("could not open pololu\n");
    perror(device);
    return 1;
  }

  set_interface_attribs (fd, B115200);  // set speed to 115,200 bps

  printf("opened Pololu device\n");

  ros::Subscriber sub = n.subscribe("/pololuCmd", 1000, pidCallback);
  ros::spin();
  
  close(fd);
  return 0;
}
