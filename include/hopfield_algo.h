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
#include <sstream>
#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

using namespace std;

struct POINT
{
    int x;
    int y;
};

struct THREAT_STRUCT
{
    POINT center;
    int range;
};

//////////////////////////////////////////////////////////////////////////
//global varibles
extern int g_MapWidth;			//width of map
extern int g_MapHeight;         	//height of map

extern int g_FlyingHeight;		//height of client move 

extern short* g_Terrain;		//rerrian map
extern int** g_SearchMap;		//search map

extern THREAT_STRUCT g_Threats[100];    //threat
extern int g_ThreatsCount;		//number of threats


extern int g_Sx;			//start point x
extern int g_Sy;			//start point y
extern int g_Ex;			//end point x
extern int g_Ey;			//end point y
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//get the threats and the configure parameters
bool InitThreat();
//get the terrian map
void InitTerrain();
//get the search map
void BuildSearchMap();
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//initialize
bool InitData();
//search
bool Search();

int GetRoad(POINT** Road);
//clean the RAM
void Clean();
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////

//
void ConnectionSearch();
//build the field
void FieldBuilding();
//roadsearch
void RoadSearch();
//////////////////////////////////////////////////////////////////////////


