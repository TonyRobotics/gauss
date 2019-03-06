/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO

#include <ros/ros.h>
#include <dynamixel_workbench_msgs/WheelCommand.h>

#define ESC_ASCII_VALUE             0x1b
#define FORWARD                     0x77
#define BACKWARD                    0x78
#define LEFT                        0x61
#define RIGHT                       0x64
#define STOPS                       0x73

int getch(void)
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "wheel_operator");

  ROS_INFO("Set angular velocity(+-0.2 rad/sec) to your Dynamixel!! by using keyboard");
  ROS_INFO("w : Forward");
  ROS_INFO("x : Backward");
  ROS_INFO("a : Left");
  ROS_INFO("d : Right");
  ROS_INFO("s : STOPS\n");
  ROS_INFO("ESC : exit");

  dynamixel_workbench_msgs::WheelCommand wheel_command;
  ros::NodeHandle node_handle;

  ros::ServiceClient wheel_command_client =
                node_handle.serviceClient<dynamixel_workbench_msgs::WheelCommand>("/wheel_command");
  ros::Rate loop_rate(250);

  while(ros::ok())
  {
    if (kbhit())
    {
      char c = getch();

      if (c == ESC_ASCII_VALUE)
      {
        break;
      }

      if (c == FORWARD)
      {
        wheel_command.request.right_vel += 0.2;
        wheel_command.request.left_vel  += 0.2;
      }
      else if (c == BACKWARD)
      {
        wheel_command.request.right_vel += -0.2;
        wheel_command.request.left_vel  += -0.2;
      }
      else if (c == LEFT)
      {
        wheel_command.request.right_vel += 0.2;
        wheel_command.request.left_vel  += -0.2;
      }
      else if (c == RIGHT)
      {
        wheel_command.request.right_vel += -0.2;
        wheel_command.request.left_vel  += 0.2;
      }
      else if (c == STOPS)
      {
        wheel_command.request.right_vel = 0.0;
        wheel_command.request.left_vel  = 0.0;
      }
      else
      {
        wheel_command.request.right_vel += 0.2;
        wheel_command.request.left_vel  += 0.2;
      }

      if (wheel_command_client.call(wheel_command))
      {
        if (wheel_command.response.result)
          ROS_INFO("Succeed to write goal_velocity");
        else
          ROS_WARN("Failed to write goal_velocity");
      }
      else
      {
        ROS_ERROR("Failed to call service /wheel_command");
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
