#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <chrono>
#include <iostream>
#include <libusbp.hpp>
#include <memory>
#include <streambuf>
#include <thread>
#include "ros/ros.h"
#include <msg_pkg/Controller.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define STM_VID 0x0483
#define STM_PID 0x5740

uint8_t pc_to_stm[17] = {'i', 't', 's'};
int serial_port;
int invalidDataCount = 0;
ros::Time start_time;
ros::Subscriber sub_joy;

msg_pkg::Controller controller_msg;
bool isCommunicationActive = true;

void reset_usb_stm();
void communicator_callback(const ros::TimerEvent &event);
void velCallback(const msg_pkg::Controller::ConstPtr &vel_msg);
int initialize_tty(const char *dev_path);
std::string discover_STM(uint16_t vid, uint16_t pid);

int main(int argc, char **argv)
{
ros::init(argc, argv, "serial_node");
printf("Serial node started\n");
ros::NodeHandle nh;
start_time = ros::Time::now();

std::string dev_path = discover_STM(STM_VID, STM_PID);
if (dev_path != "NULL")
{
serial_port = initialize_tty(dev_path.c_str());
printf("Master Romusa discovered at %s\n", dev_path.c_str());
}
else
{
while (dev_path == "NULL")
{
printf("Master Romusa not found, retrying...\n");
dev_path = discover_STM(STM_VID, STM_PID);
std::this_thread::sleep_for(std::chrono::seconds(1));
}
serial_port = initialize_tty(dev_path.c_str());
printf("Master Romusa discovered at %s\n", dev_path.c_str());
}

printf("sizeof pc_to_stm: %ld\n", sizeof(pc_to_stm));

reset_usb_stm();

// Start timers
ros::Timer timer = nh.createTimer(ros::Duration(0.001), communicator_callback);
sub_joy = nh.subscribe<msg_pkg::Controller>("Controller", 10, velCallback);

ROS_INFO("Subscribing to /Controller");

ros::spin();
return 0;
}

void communicator_callback(const ros::TimerEvent &event)
{
ros::Time current_time = ros::Time::now();

uint16_t check_sum = 0;
for (int i = 3; i < 15; i++)
{
check_sum += pc_to_stm[i];
}
check_sum = ~check_sum;

memcpy(pc_to_stm + 15, &check_sum, 2);
write(serial_port, pc_to_stm, 17);
}

void velCallback(const msg_pkg::Controller::ConstPtr &vel_msg)
{
memcpy(pc_to_stm + 3, &vel_msg->velx, 4);
memcpy(pc_to_stm + 7, &vel_msg->vely, 4);
memcpy(pc_to_stm + 11, &vel_msg->angvel, 4);
}

std::string discover_STM(uint16_t vid, uint16_t pid)
{
std::string dev_path;
libusbp::device device = libusbp::find_device_with_vid_pid(vid, pid);
if (!device)
{
return "NULL";
}
else
{
try
{
libusbp::serial_port port(device, 0, true);
dev_path = port.get_name();
return dev_path;
}
catch (const std::exception &error)
{
std::cerr << "Exception " << error.what() << std::endl;
return "NULL";
}
}
return "NULL";
}

int initialize_tty(const char *dev_path)
{
int serial_port = open(dev_path, O_RDWR);
if (serial_port < 0)
{
std::cerr << "Error " << errno << " from open:" << strerror(errno) << std::endl;
return -1;
}

struct termios tty;
if (tcgetattr(serial_port, &tty) != 0)
{
std::cerr << "Error " << errno << " from tcgetattr:" << strerror(errno) << std::endl;
return -1;
}

tty.c_cflag &= ~PARENB;
tty.c_cflag &= ~CSTOPB;
tty.c_cflag &= ~CSIZE;
tty.c_cflag |= CS8;
tty.c_cflag |= CRTSCTS;
tty.c_cflag |= CREAD | CLOCAL;

tty.c_lflag &= ~ICANON;
tty.c_lflag &= ~ECHO;
tty.c_lflag &= ~ECHOE;
tty.c_lflag &= ~ISIG;

tty.c_iflag &= ~(IXON | IXOFF | IXANY);
tty.c_iflag &= ~(ICRNL | INLCR);

tty.c_oflag &= ~OPOST;
tty.c_oflag &= ~ONLCR;

tty.c_cc[VMIN] = 1;
tty.c_cc[VTIME] = 0;

cfsetispeed(&tty, B115200);
cfsetospeed(&tty, B115200);

if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
{
std::cerr << "Error " << errno << " from tcsetattr:" << strerror(errno) << std::endl;
return -1;
}
return serial_port;
}

void reset_usb_stm()
{
float initial_speed[3] = {0, 0, 0};
memcpy(pc_to_stm + 3, initial_speed, 12);

}

