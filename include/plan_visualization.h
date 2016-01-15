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
//--------------------------------------------
/*

	plan for RViz v1.0
				
				
*/
//--------------------------------------------

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

#define max(x,y) (x>y?x:y)

//parameter for the map reducing ratio
extern float AltitudeZoom;

//--------------------------------------------
extern ros::Publisher Marker_Publisher;		
extern visualization_msgs::Marker Tail;		//path
extern visualization_msgs::Marker Terrain; 	//terrian（ID=999）
extern visualization_msgs::Marker TerrainLine; 	//terrianline（ID=998）
//--------------------------------------------

//initialize the publisher
void InitMarkerPublisher();
//chooss the color based on the ID
void GetStoreColor(int ID,float& r,float& g,float& b);
//intialize the marker
void UInitMarker(visualization_msgs::Marker& marker);

//draw the cylinder
void Draw_Cylinder(int ID,float x,float y,float z,float Rad,float Height,float r=1.0f,float g=1.0f,float b=1.0f,float a=1.0f);
//draw the ball
void Draw_Sphere(int ID,float x,float y,float z,float Rad,float r=1.0f,float g=1.0f,float b=1.0f,float a=1.0f);

//initialize the path
void InitTail(int ID,float r=1.0f,float g=1.0f,float b=1.0f,float a=1.0f);
//update the path
void AddTailPoint(float x,float y,float z);
//clean the path
void ResetTail();

//draw the terrian
void DrawTerrain(int Width,int Height,int space,short* TerrainMap);
