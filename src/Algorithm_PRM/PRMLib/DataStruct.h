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
#ifndef DATASTRUCT_H_
#define DATASTRUCT_H_

#include <cstdlib>
#include <stdlib.h>
#include <list>
#include "../algo.h"

//##ModelId=4917DD9901F6
struct Vertex
{
	//##ModelId=4917DD990204
	int x ;
	//##ModelId=4917DD990205
	int y ;
	//##ModelId=4917DD990206
	double longitude ;
	//##ModelId=4917DD990213
	double latitude ;
	//##ModelId=4917DD990214
	double altitude ;
	//##ModelId=4917DD990215
	Vertex(int X=0, int Y=0, double L=0, double B=0, double H=0)
	{
		x = X ;
		y = Y ;
		longitude = L ;
		latitude  = B ;
		altitude  = H ;
	}
} ;

//##ModelId=4917DD990224
struct Edge
{
	//##ModelId=4917DD990232
	int vertex_index1 ;
	//##ModelId=4917DD990233
	int vertex_index2 ;

	//##ModelId=4917DD990242
	Edge(int index1=-1, int index2=-1)
	{
		vertex_index1 = index1 ;
		vertex_index2 = index2 ;
	}
} ;
//##ModelId=4917DD990245
typedef std::list<Edge> CEdgeList ;


//data struct for the ant colony optimization,ACO
//##ModelId=4917DD990252
struct C_City
{
	//##ModelId=4917DD990254
	int index ;
	//##ModelId=4917DD990261
	int x ;
	//##ModelId=4917DD990262
	int y ;
	//##ModelId=4917DD990263
	double longitude ;
	//##ModelId=4917DD990271
	double latitude ;
	//##ModelId=4917DD990272
	double altitude ;

	//##ModelId=4917DD990273
	C_City(int index=0, int X=0, int Y=0, double L=0, double B=0, double H=0)
	{
		index = index ;
		x = X ;
		y = Y ;
		longitude = L ;
		latitude = B ;
		altitude = H ;
	}
} ;
// path defination based on the city struct
//##ModelId=4917DD990283
typedef std::list<C_City> CRoute ;


// data struct for the ACO
//##ModelId=4917DD990291
struct C_Ant
{
	//##ModelId=4917DD990293
	int AntIndex ;
	//##ModelId=4917DD99029F
	int VertexIndex ;
	//##ModelId=4917DD9902A0
	int Position_X ;
	//##ModelId=4917DD9902A1
	int Position_Y ;
	//##ModelId=4917DD9902AF
	int AtTime ;
	//##ModelId=4917DD9902BF
	int ErstTime ;
	//##ModelId=4917DD9902C0
	int LastTime ;
	//##ModelId=4917DD9902C2
	CRoute Route ;
	//##ModelId=4917DD9902CE
	double Length ;
	//##ModelId=4917DD9902CF
	double Menace ;
	//##ModelId=4917DD9902DE
	double F ;

	//##ModelId=4917DD9902DF
	C_Ant(int Index1=-1, int Index2=-1, int X=0, int Y=0, int Time=0, int Time_E=0, int Time_L=0)
	{
		AntIndex = Index1 ;
		VertexIndex = Index2 ;
		Position_X = X ;
		Position_Y = Y ;
		AtTime = Time ;
		ErstTime = Time_E ;
		LastTime = Time_L ;
		Route.clear() ;
		Length = 0 ;
		Menace = 0 ;
		F = 0 ;
	}
} ;


// data struct for the threat
//##ModelId=4917DD9902F1
struct C_Menace
{
	//##ModelId=4917DD9902FE
	int MenaceID ;
	//##ModelId=4917DD9902FF
	int Position_X ;
	//##ModelId=4917DD990300
	int Position_Y ;
	//##ModelId=4917DD99030D
	double longitude ;
	//##ModelId=4917DD99030E
	double latitude ;
	//##ModelId=4917DD99030F
	double radius ;
	//##ModelId=4917DD99031C
	double value ;
} ;
// list for the threat
//##ModelId=4917DD99031D
typedef std::list<C_Menace> CMenaceList ;

#endif
