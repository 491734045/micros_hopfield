/*
* Software License Agreement (BSD License)
*Copyright (c) 2015, micROS Team
 http://micros.nudt.edu.cn/
*All rights reserved.
* Copyright (c) 2009, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "micros_hopfield/plan.h"
#include "plan_visualization.h"

using namespace std;

int main(int argc, char **argv)
{
  //flight height（the data resolution ration is 90）
  float g_FlyingHeight=650.0/90;
  
  srand((int)time(0));
  
  //if lose parameter
  if (argc != 2)
  {
    ROS_INFO("usage: Plan_client NodeID");
    return 1;
  }

  //get the client ID
  int NodeID=atoi(argv[1]);
  if (NodeID <=0)
  {
    ROS_INFO("usage: Plan_client NodeID (NodeID must greater than 0)");
    return 1;
  }  

  char Name[255];
  sprintf(Name,"X%d",NodeID);
  ros::init(argc, argv, Name); 
 
  ros::NodeHandle n;

  //service of plan 
  ros::ServiceClient client = n.serviceClient<micros_hopfield::plan>("Planner");
  micros_hopfield::plan srv;
	
  //publish the rviz
  InitMarkerPublisher();

  //the color of client in rviz
  float r=1.0,g=1.0,b=1.0;
  GetStoreColor(NodeID,r,g,b);

  //initialize
  InitTail(100+NodeID,r,g,b);

  //initial starting point(1,1)
  srv.request.sx = 1;
  srv.request.sy = 1;
  bool Finded=false;

   while (1)
  {
 	 srv.request.ex = int(1200.0*rand()/RAND_MAX);
  	 srv.request.ey = int(1200.0*rand()/RAND_MAX);
	
	 Draw_Sphere(200+NodeID,srv.request.ex,srv.request.ey,g_FlyingHeight,4,r,g,b,0.5);

	 ROS_INFO("Plan from (%ld,%ld) to (%ld,%ld)", 	(long int)srv.request.sx, (long int)srv.request.sy,
							(long int)srv.request.ex, (long int)srv.request.ey);
	 struct timeval t_start,t_end;
	 gettimeofday(&t_start,NULL);

	 if (client.call(srv))
	 {
	    ROS_INFO("Total Points: %ld", (long int)srv.response.Route_Points);
	 }
	 else
	 {
	    ROS_ERROR("Failed to call service HopField");
	    continue;
	 }

   if (srv.response.Route_Points <= 0 )
   {
     ROS_WARN("No Solution! Reset Target Point");
   }

	 int m_nStep=srv.response.Route_Points;
	 if ((m_nStep>0)&&(m_nStep<=10000))
	 {
		ros::Rate loop_rate(m_nStep);

	 	gettimeofday(&t_end,NULL);
		long cost_time=abs(t_end.tv_usec-t_start.tv_usec);
		ROS_INFO("--Cost Time: %f s--", cost_time*1.0/1000000);	
		FILE* fp = fopen("TimeConsume.txt","a");
		fprintf(fp,"time : %f\n",cost_time*1.0/1000);
		fprintf(fp,"step : %d\n",m_nStep);
		fclose(fp);		

		ResetTail();
 		for (int i=0;i<m_nStep;i++)
	 	{
			Draw_Sphere(300+NodeID,srv.response.Route_X[i],srv.response.Route_Y[i],g_FlyingHeight,4,r,g,b,1.0);//red ball stands for the client
	    		AddTailPoint(srv.response.Route_X[i],srv.response.Route_Y[i],g_FlyingHeight);
	   		loop_rate.sleep();
		}
 		srv.request.sx =  srv.request.ex;
	 	srv.request.sy =  srv.request.ey;  
		Finded=true;
 	 }
	
	 if (!Finded)
	 {
 		srv.request.sx = int(1200.0*rand()/RAND_MAX);
  	 	srv.request.sy = int(1200.0*rand()/RAND_MAX);
	}
  }

  return 0;
}
