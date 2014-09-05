/*
 *
 *  Starts Blender using a system() call, so that it can be
 *  started from a roslaunch file.
 *
 *  Date: September 2012
 *  Authors: David Butterworth
 *
 *  Date: August 2014
 *  Authors: Jamie Diprose
 *
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * Copyright (c) 2014, Jamie Diprose, OpenCog
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include <string>
#include <cstdlib>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>

using namespace std;

/*
Define Blender executable name and parameters to automatically start Blender Game Engine.
The full path will come from the PATH environment variable.

Blender command line arguments:
   (http://wiki.blender.org/index.php/Doc:2.6/Manual/Render/Command_Line)
*/

string blender_executable = "blender";
string blender_game_engine = "--engine BLENDER_GAME";
string enable_autoexec = "--enable-autoexec";
int status;
pid_t pid;
string blend_file;
string full_cmd;
string python_script;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blender");
    ros::NodeHandle nh("~");

    if (nh.hasParam("blend_file"))
    {
        bool use_game_engine;
        nh.param<bool>("use_game_engine", use_game_engine, false);

        if(use_game_engine)
        {
            nh.getParam("blend_file", blend_file);
            full_cmd = blender_executable + " " + blender_game_engine + " " + enable_autoexec + " " + blend_file;
        }
        else
        {
            if(nh.hasParam("python_script"))
            {
                nh.getParam("python_script", python_script);
                nh.getParam("blend_file", blend_file);
                full_cmd = blender_executable + " " + blend_file + " --background --python " + python_script + " ";
            }
            else
            {
                ROS_ERROR("Error python_script not specified, exiting");
                return EXIT_FAILURE;
            }
        }

        ROS_INFO("Starting Blender...");
        ROS_INFO(full_cmd.c_str());

        pid = fork();
        if (pid == 0)
        {
            // Child process, start Blender
            system(full_cmd.c_str());
        }
        else if (pid > 0)
        {
            // Parent process, wait until ROS exits, then kill blender
            ros::spin();
            string kill_cmd = "pkill -9 -f " + blend_file;
            system(kill_cmd.c_str());
            kill(pid, SIGKILL); //kill child process
        }
        else
        {
            // Forking failed
            ROS_ERROR("Failed to fork process, exiting");
            return EXIT_FAILURE;
        }
    }
    else
    {
        ROS_ERROR("Error no blend file specified, exiting");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
