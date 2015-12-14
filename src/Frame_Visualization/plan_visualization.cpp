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
#include "plan_visualization.h"

//parameter for the map reducing ratio
float AltitudeZoom=3.0;

//--------------------------------------------
ros::Publisher Marker_Publisher;		
visualization_msgs::Marker Tail;		//path
visualization_msgs::Marker Terrain; 		//terrian（ID=999）
visualization_msgs::Marker TerrainLine; 	//terrianline（ID=998）
//--------------------------------------------

//initialize the publisher
void InitMarkerPublisher()
{
	ros::NodeHandle n;
	Marker_Publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
}

//chooss the color based on the ID
void GetStoreColor(int ID,float& r,float& g,float& b)
{
	switch (ID)
  	{
    		case 1: r=1.0; g=0.0; b=0.0;	break;
    		case 2: r=0.0; g=0.0; b=1.0;	break;
    		case 3: r=0.0; g=1.0; b=1.0;	break;
    		case 4: r=1.0; g=1.0; b=0.0; 	break;
    		case 5: r=1.0; g=0.0; b=1.0;	break;
    		default:r=1.0; g=1.0; b=1.0;	break;
  	}
}

//intialize the marker
void UInitMarker(visualization_msgs::Marker& marker)
{
	marker.header.frame_id = "/my_frame";
  	marker.header.stamp = ros::Time::now();
  	marker.ns = "HopField_client";
  	marker.action = visualization_msgs::Marker::ADD;
 	marker.pose.orientation.w = 1.0;
	marker.scale.x =1;
	marker.scale.y =1;
	marker.scale.z = 1.0;
	marker.color.r=1.0;
	marker.color.g=1.0;
	marker.color.b=1.0;
	marker.color.a=1.0;
}

//draw the cylinder
void Draw_Cylinder(int ID,float x,float y,float z,float Rad,float Height,float r,float g,float b,float a)
{
	visualization_msgs::Marker cylinder;
  	UInitMarker(cylinder);

  	cylinder.id = ID;
	cylinder.type = visualization_msgs::Marker::CYLINDER;
  	
	cylinder.pose.position.x = x;
  	cylinder.pose.position.y = y;
	cylinder.pose.position.z = z;
	
 	cylinder.scale.x =2*Rad;
  	cylinder.scale.y =2*Rad;
	cylinder.scale.z =Height;

	cylinder.color.r = r;
	cylinder.color.g = g;
	cylinder.color.b = b;
	cylinder.color.a = a;

  	// Publish the marker
	for (int i=0;i<2;i++)
  	{
		Marker_Publisher.publish(cylinder);
  		ros::Rate loop_rate(5);
		loop_rate.sleep();
	}
}

//draw the ball
void Draw_Sphere(int ID,float x,float y,float z,float Rad,float r,float g,float b,float a)
{
	visualization_msgs::Marker sphere;
	UInitMarker(sphere);
  
	sphere.id = ID;
	sphere.type = visualization_msgs::Marker::SPHERE;
	 
	sphere.pose.position.x = x;
	sphere.pose.position.y = y;
	sphere.pose.position.z = z*AltitudeZoom;
 
  	sphere.scale.x =2*Rad;
  	sphere.scale.y =2*Rad;
  	sphere.scale.z =2*Rad;

  	// Set the color -- be sure to set alpha to something non-zero!
  	sphere.color.r = r;
  	sphere.color.g = g;
  	sphere.color.b = b;
  	sphere.color.a = a;

  	// Publish the marker
  	Marker_Publisher.publish(sphere);
}

//initialize the path
void InitTail(int ID,float r,float g,float b,float a)
{
	UInitMarker(Tail);

  	Tail.id = ID;
  	Tail.type = visualization_msgs::Marker::LINE_STRIP;

	Tail.scale.x=2;

  	Tail.color.r = r;
  	Tail.color.g = g;
  	Tail.color.b = b;
  	Tail.color.a = a;
}

//update the path
void AddTailPoint(float x,float y,float z)
{
 	geometry_msgs::Point p;
  	p.x = x;
  	p.y = y;
  	p.z = z*AltitudeZoom;

  	Tail.points.push_back(p);
  	Marker_Publisher.publish(Tail);
}

//clean the path
void ResetTail()
{
	Tail.points.clear();
}


#define TERRAIN_PUSH_POINT(i,j) {p.x=(j)*space;p.y=(i)*space;p.z=max(0,TerrainMap[(i)*space*Width+(j)*space]*AltitudeZoom/90);Terrain.points.push_back(p);}
#define TERRAINLINE_PUSH_POINT(i,j) {p.x=(j)*space;p.y=(i)*space;p.z=max(0,TerrainMap[(i)*space*Width+(j)*space]*AltitudeZoom/90+1);TerrainLine.points.push_back(p);}

//draw the terrian
void DrawTerrain(int Width,int Height,int space,short* TerrainMap)
{
	if (TerrainMap==NULL) return;

 	UInitMarker(Terrain);
	UInitMarker(TerrainLine);

  	Terrain.id = 999;
  	Terrain.type = visualization_msgs::Marker::TRIANGLE_LIST;
	TerrainLine.id = 998;
  	TerrainLine.type = visualization_msgs::Marker::LINE_LIST;

	int w=Width/space;
	int h=Height/space;

	if ((w==0)||(h==0)) return;  

 	 // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  	geometry_msgs::Point p;
	for (int i=0;i<h-1;i++)
  	{
		for (int j=0;j<w-1;j++)
   		{
			TERRAIN_PUSH_POINT(i ,j);
			TERRAIN_PUSH_POINT(i+1,j);
			TERRAIN_PUSH_POINT(i+1,j+1);
			TERRAIN_PUSH_POINT(i ,j);
			TERRAIN_PUSH_POINT(i,j+1);
			TERRAIN_PUSH_POINT(i+1,j+1);

			TERRAINLINE_PUSH_POINT(i ,j);
			TERRAINLINE_PUSH_POINT(i+1,j);
			TERRAINLINE_PUSH_POINT(i ,j);
			TERRAINLINE_PUSH_POINT(i,j+1);
   		}
	}

	Terrain.color.r = 0.0f;
	Terrain.color.b = 0.0f;
	Terrain.color.a = 0.5f;

	ros::Rate loop_rate(5);
	for (int i=0;i<3;i++)
	{
		Marker_Publisher.publish(Terrain);
	  	loop_rate.sleep();
		Marker_Publisher.publish(TerrainLine);
	  	loop_rate.sleep();
	}
}




















