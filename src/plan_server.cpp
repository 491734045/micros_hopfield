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
#include "ros/ros.h"

#include "micros_hopfield/plan.h"

#include "plan_visualization.h"


#include "hopfield_algo.h"

bool plan(micros_hopfield::plan::Request  &req,micros_hopfield::plan::Response &res )
{
    //----------------------------------------------
    //initialize
    g_Sx=req.sx;
    g_Sy=req.sy;
    g_Ex=req.ex;
    g_Ey=req.ey;

    if (g_SearchMap[g_Ey][g_Ex]<0) 
    {
	     res.Route_Points=0;
      	return true;
    }

    bool bInitDataRet=InitData();
    if (bInitDataRet)
        cout<<"Data Inited OK!"<<endl;
    else
        cout<<"Data Inited Error!"<<endl;

    if (Search())
        cout<<"search finish!"<<endl;
    else
    {
        cout<<"I can't find the path!"<<endl;
 	      res.Route_Points=0;
        return true;
    }

    //save the path
    POINT* m_Step;				
    int m_nStep=GetRoad(&m_Step);
    if (m_nStep>0)
    {
 	res.Route_X.clear();
	res.Route_Y.clear();
	for (int i=0;i<m_nStep;i++)
        {
           res.Route_X.push_back(m_Step[i].x);
    	   res.Route_Y.push_back(m_Step[i].y);
        }
        res.Route_Points=m_nStep;
	
	delete [] m_Step;
    }
  	
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Plan_server");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("Planner", plan);
  ROS_INFO("Ready to Plan.");

  //publish the marker in rviz
  InitMarkerPublisher();

  //input the threat position
  if (!InitThreat()) 
 	return false; 
  //Draw the threat area in the rviz
  for (int j=0;j<g_ThreatsCount;j++)
	Draw_Cylinder( 3+j,g_Threats[j].center.x,g_Threats[j].center.y,25,g_Threats[j].range,50,0.0,1.0,1.0,1.0);
  //input the terrain description file
  InitTerrain(); 
  //draw the terrain in rviz
  DrawTerrain(g_MapWidth,g_MapHeight,4,g_Terrain);
  
  BuildSearchMap();

  ros::spin();

  return 0;
}
