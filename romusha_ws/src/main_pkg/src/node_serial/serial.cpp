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
// #include <romusa_messages/odom.h>
// #include <romusa_messages/cmd_vel.h>
// #include <romusa_messages/cmd_slider.h>
// #include <romusa_messages/vac.h>
// #include <romusa_messages/enc_slider.h>

// Linux headers
#include <errno.h>   // Error integer and strerror() function
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#define STM_VID 0x0483
#define STM_PID 0x5740

// uint8_t stm_to_pc[84];
uint8_t pc_to_stm[17] = {'i', 't', 's'};

// uint8_t stm_to_pc_copy[84];
// uint8_t pc_to_stm_copy[43] = {'i', 't', 's'};

int serial_port;
int invalidDataCount = 0;
// uint32_t pub_bandwidth = 0;

// uint16_t test_adc[16];

ros::Time start_time;

// ros::Publisher pub_odom;
// ros::Publisher pub_encSlider;

// ros::Subscriber sub_vel;
// ros::Subscriber sub_slider;
// ros::Subscriber sub_vacuum;

// romusa_messages::odom odom_msg;
// romusa_messages::enc_slider enc_slider_msg;

ros::Subscriber sub_joy;

msg_pkg::Controller;



bool isCommunicationActive = true;

void reset_usb_stm();

void communicator_callback(const ros::TimerEvent &event);
void transfer_monitor_callback(const ros::TimerEvent &event);
// void publisher_callback(const ros::TimerEvent &event);

void velCallback(const romusa_messages::cmd_vel::ConstPtr &vel_msg);
// void sliderCallback(const romusa_messages::cmd_slider::ConstPtr &slider_msg);
// void vacCallback(const romusa_messages::vac::ConstPtr &vac_msg);

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
    // serial_port = initialize_tty(dev_path.c_str());
    serial_port = initialize_tty("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_20913786424D-if00");
    printf("Master Romusa discovered at %s\n", dev_path.c_str());
  }
  else
  {
    while (dev_path == "NULL")
    {
      printf("Master Romusa not found, retry!!!\n");
      dev_path = discover_STM(STM_VID, STM_PID);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    serial_port = initialize_tty("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_20913786424D-if00");
    printf("Master Romusa discovered at %s\n", dev_path.c_str());
  }
  // printf("sizeof stm_to_pc: %ld\n", sizeof(stm_to_pc));
  printf("sizeof pc_to_stm: %ld\n", sizeof(pc_to_stm));

  reset_usb_stm();

  // start timers
  ros::Timer timer = nh.createTimer(ros::Duration(0.001), communicator_callback);
  // ros::Timer timer2 = nh.createTimer(ros::Duration(1), transfer_monitor_callback);
  // ros::Timer timer3 = nh.createTimer(ros::Duration(0.002), publisher_callback);

  // pub_odom = nh.advertise<romusa_messages::odom>("odom", 100);
  // pub_encSlider = nh.advertise<romusa_messages::enc_slider>("encSlider", 100);

  // sub_vel = nh.subscribe<romusa_messages::cmd_vel>("velocity", 100, velCallback);
  // sub_slider = nh.subscribe<romusa_messages::cmd_slider>("slider", 100, sliderCallback);
  // sub_vacuum = nh.subscribe<romusa_messages::vac>("vacuum", 100, vacCallback);

  sub_joy = nh.subscribe<romusa_messages::cmd_vel>("Controller", 10, velCallback);

  // ROS_INFO("Publishing to /odom, /encSlider");
  // ROS_INFO("Subscribing to /velocity, /slider, /vacuum");

  ROS_INFO("Subscribing to /velocity");

  // ros::MultiThreadedSpinner spinner(4);
  // spinner.spin();
  ros::spin();
  return 0;
}

void communicator_callback(const ros::TimerEvent &event)
{
  // Get the current time
  ros::Time current_time = ros::Time::now();

  uint8_t buffer[84];
  // Fetch to_stm data and copy to pc_to_stm to be send to STM32
  uint16_t check_sum = 0;
  for (int i = 3; i < 15; i++)
  {
    check_sum += pc_to_stm[i];
  }
  check_sum = ~check_sum;

  memcpy(pc_to_stm + 15, &check_sum, 2);

  write(serial_port, pc_to_stm, 17);

  // // Read data from STM32 after sending, and store it to stm_to_pc
  // int num_bytes = read(serial_port, buffer, 84);
  // // memcpy(stm_to_pc, buffer, 84);
  // uint16_t check_sum_stm = 0;
  // for (int i = 3; i < 82; i++)
  // {
  //   check_sum_stm += buffer[i];
  // }
  // check_sum_stm = ~check_sum_stm;
  // uint16_t check_sum_stm_read;
  // memcpy(&check_sum_stm_read, buffer + 82, 2);
  // // if(check_sum_stm == check_sum_stm_read){
  // if (num_bytes == 84)
  // {
  //   invalidDataCount = 0;
  //   isCommunicationActive = true; // Communication is active
  //   // memcpy(stm_to_pc, buffer, 84);
  // }
  // else
  // {
  //   invalidDataCount++;
  //   printf("Data Invalid count : %d\r\n", invalidDataCount);
  //   // memset(stm_to_pc, 0, 84); // This already sets the data to zero if the data is invalid.

  //   if (invalidDataCount >= 10) // If invalid data is received 10 times in a row.
  //   {
  //     isCommunicationActive = false; // Communication is considered lost
  //     // Attempt to rediscover and reconnect to the device.
  //     std::string dev_path = discover_STM(STM_VID, STM_PID);
  //     if (dev_path == "NULL") // Device is not found.
  //     {
  //       std::cerr << "Can't find Romusa\r\n";
  //       // Since the device is not found, ensure all data is explicitly set to zero.
  //       // memset(stm_to_pc, 0, 84); // Ensure this line is here to set everything to zero when device is not found.
  //     }
  //     else
  //     {
  //       serial_port = initialize_tty("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_20913786424D-if00");
  //       // Optionally, reset invalidDataCount after reinitialization to start the count over.
  //       invalidDataCount = 0;
  //     }
  //   }
  // }
}

void velCallback(const romusa_messages::cmd_vel::ConstPtr &vel_msg)
{
  memcpy(pc_to_stm + 3, &vel_msg->velX, 4);
  memcpy(pc_to_stm + 7, &vel_msg->velY, 4);
  memcpy(pc_to_stm + 11, &vel_msg->angvel, 4);
}

// void transfer_monitor_callback(const ros::TimerEvent &event)
// {
//   // Get the current time
//   ros::Time current_time = ros::Time::now();

//   // Calculate the time elapsed in milliseconds
//   double elapsed_time_ms = (current_time - start_time).toSec() * 1000;
//   printf("%f Publisher with Bandwidth %dHz\r\n", elapsed_time_ms, pub_bandwidth);
//   pub_bandwidth = 0;
// }

// void publisher_callback(const ros::TimerEvent &event)
// {
//    if (!isCommunicationActive)
//   {
//   memset(&odom_msg, 0, sizeof(odom_msg)); // Assuming odom_msg is a struct, set all its fields to zero
//   memset(&enc_slider_msg, 0, sizeof(enc_slider_msg)); // Assuming enc_slider_msg is a struct, set all its fields to zero

//   // Consider publishing the zeroed or safe-value data here if needed
//   pub_odom.publish(odom_msg);
//   pub_encSlider.publish(enc_slider_msg);
//     return; // Skip publishing
//   }
//   // Get the current time
  
//   ros::Time current_time = ros::Time::now();

//   // Calculate the time elapsed in milliseconds
//   double elapsed_time_ms = (current_time - start_time).toSec() * 1000;

//   memcpy(&odom_msg.posX, stm_to_pc + 3, 4);
//   memcpy(&odom_msg.posY, stm_to_pc + 7, 4);
//   memcpy(&odom_msg.posTheta, stm_to_pc + 11, 4);
//   memcpy(&odom_msg.posLocal_X, stm_to_pc + 15, 4);
//   memcpy(&odom_msg.posLocal_Y, stm_to_pc + 19, 4);
//   memcpy(&odom_msg.tick, stm_to_pc + 23, 4);
//   memcpy(&enc_slider_msg.encArm, stm_to_pc + 27, 2);
//   memcpy(&enc_slider_msg.encLifter, stm_to_pc + 29, 2);
//   memcpy(&enc_slider_msg.trigButton, stm_to_pc + 31, 1);
//   memcpy(&enc_slider_msg.retryButton, stm_to_pc + 32, 1);
//   memcpy(&enc_slider_msg.mapButton, stm_to_pc + 33, 1);

//   memcpy(&enc_slider_msg.proxKanan, stm_to_pc + 34, 1);
//   memcpy(&enc_slider_msg.proxKiri, stm_to_pc + 35, 1);

//   memcpy(test_adc, stm_to_pc + 36, 32);
//   enc_slider_msg.sensor[0] = test_adc[3];
//   enc_slider_msg.sensor[3] = test_adc[6];
//   enc_slider_msg.sensor[4] = test_adc[7];
//   enc_slider_msg.sensor[5] = test_adc[14];
//   enc_slider_msg.sensor[6] = test_adc[15];
//   enc_slider_msg.sensor[7] = test_adc[8];
//   enc_slider_msg.sensor[9] = test_adc[2];
//   enc_slider_msg.sensor[10] = test_adc[1];
//   enc_slider_msg.sensor[11] = test_adc[0];
//   enc_slider_msg.sensor[12] = test_adc[13];
//   enc_slider_msg.sensor[13] = test_adc[12];
//   enc_slider_msg.sensor[15] = test_adc[10];

//   pub_odom.publish(odom_msg);
//   pub_encSlider.publish(enc_slider_msg);

//   pub_bandwidth++;
// }

// void sliderCallback(const romusa_messages::cmd_slider::ConstPtr &slider_msg)
// {
//   memcpy(pc_to_stm + 15, &slider_msg->target_arm, 2);
//   memcpy(pc_to_stm + 17, &slider_msg->limit_arm, 2);
//   memcpy(pc_to_stm + 19, &slider_msg->target_lifter, 2);
//   memcpy(pc_to_stm + 21, &slider_msg->limit_lifter, 2);
// }

// void vacCallback(const romusa_messages::vac::ConstPtr &vac_msg)
// {
//   memcpy(pc_to_stm + 23, &vac_msg->stat_vacuum, 1);
//   memcpy(pc_to_stm + 24, &vac_msg->stat_pneumatic, 1);
//   memcpy(pc_to_stm + 25, &vac_msg->stat_pump, 1);
//   memcpy(pc_to_stm + 26, &vac_msg->stat_servo, 1);
// }
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
    for (int i = 0; i < 2; i++)
    {
      try
      {
        libusbp::serial_port port(device, i, true);
        std::string port_name = port.get_name();
        dev_path = port_name;
        return dev_path;
      }
      catch (const std::exception &error)
      {
        std::cerr << "Exception " << error.what() << std::endl;
        return "NULL";
      }
    }
  }
  // we should've not arrived here....
  return "NULL";
}

int initialize_tty(const char *dev_path)
{
  int serial_port = open(dev_path, O_RDWR);
  printf("serial_port: %d\n", serial_port);
  if (serial_port < 0)
  {
    std::cerr << "Error " << errno << " from open:" << strerror(errno) << std::endl;
    return -1;
  }
  // Create new termios struct, we call it 'tty' for convention
  // No need for "= {0}" at the end as we'll immediately write the existing
  // config to this struct
  struct termios tty;

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if (tcgetattr(serial_port, &tty) != 0)
  {
    std::cerr << "Error " << errno << " from tcgetattr:" << strerror(errno) << std::endl;
    return -1;
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag |= CRTSCTS;        // Enable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
    std::cerr << "Error " << errno << " from tcsetattr:" << strerror(errno) << std::endl;
    return -1;
  }
  printf("Opened %s serial port with fd %d\n", dev_path, serial_port);
  return serial_port;
}

void reset_usb_stm()
{
  float initial_speed[3] = {0, 0, 0};
  // short int initial_arm[2] = {0, 0};
  // short int initial_lifter[2] = {0, 0};
  // bool initial_vac_pneu[3] = {0, 0, 0};
  memcpy(pc_to_stm + 3, initial_speed, 12);
  // memcpy(pc_to_stm + 15, initial_arm, 4);
  // memcpy(pc_to_stm + 19, initial_lifter, 4);
  // memcpy(pc_to_stm + 23, initial_vac_pneu, 3);
}