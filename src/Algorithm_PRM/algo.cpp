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
#include "algo.h"
 
#include "PRMLib/DataStruct.h"
#include "PRMLib/PrmMap.h"
#include "PRMLib/ACRoutePlanner.h"

//////////////////////////////////////////////////////////////////////////
//global varibles
int g_MapWidth=-1;		//width of map
int g_MapHeight=-1;         	//height of map

int g_FlyingHeight=100;		//height of client move

short * g_Terrain=NULL;		//rerrian map
int** g_SearchMap=NULL;		//search map

THREAT_STRUCT g_Threats[100];   //threat
int g_ThreatsCount=0;		//number of threats

int g_Sx=-1;			//start point x
int g_Sy=-1;			//start point y
int g_Ex=-1;			//end point x
int g_Ey=-1;			//end point y
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//varibles for plan
CPrmMap* prmMapVar_p=NULL;		
CACRoutePlanner* ACRoutePlannerVar_p=NULL;  
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//get the threats and the configure parameters
bool InitThreat()
{
    char Temp[255];
    FILE* fp=fopen("Environment.ppf","r"); 
    if (fp!=NULL)
    {
        fscanf(fp,"%s\n",Temp);//width of map
        g_MapWidth=atoi(Temp);
        fscanf(fp,"%s\n",Temp);//height of map
        g_MapHeight=atoi(Temp);
        fscanf(fp,"%s\n",Temp);//height of client move
        g_FlyingHeight=atoi(Temp);
        fscanf(fp,"%s\n",Temp);//number of threats
        g_ThreatsCount=atoi(Temp);
        for (int i=0;i<g_ThreatsCount;i++)
        {
            fscanf(fp,"%s\n",Temp);
            g_Threats[i].center.x=atoi(Temp);
            fscanf(fp,"%s\n",Temp);
            g_Threats[i].center.y=atoi(Temp);
            fscanf(fp,"%s\n",Temp);
            g_Threats[i].range=atoi(Temp);
        }
        fclose(fp);
        cout<<"PPF File Read OK!"<<endl;
        return true;
    }

    cout<<"PPF File Open Error!"<<endl;
    return false;
}

//get the terrian map
void InitTerrain()
{
	if (g_Terrain!=NULL) delete g_Terrain;
	int DataSize=g_MapWidth*g_MapHeight;
	g_Terrain = new short[DataSize];
	FILE* fp = fopen("Terrain.raw","rb");
	if (fp)
	{
		if (fread(g_Terrain,DataSize*2,1,fp)==DataSize*2);
		{
			fclose(fp);
			cout<<"Terrain File Open OK!"<<endl;
			return ;
		}
	}
	if (g_Terrain!=NULL) delete g_Terrain;
	g_Terrain=NULL;
	cout<<"Terrain File Open Error!"<<endl;
}

//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
//function for port

//clean the RAM
void Clean()
{
	//delete 
	if (prmMapVar_p!=NULL)
	{
		delete prmMapVar_p;
		prmMapVar_p=NULL;
	}
	if (ACRoutePlannerVar_p!=NULL)
	{
		delete ACRoutePlannerVar_p;
		ACRoutePlannerVar_p=NULL;
	}
}

//initialize
bool InitData()
{
	InitThreat();

  	if (g_ThreatsCount<0) return false;
	if (g_MapWidth<0) return false;
	if (g_MapHeight<0) return false;
	
	//////////////////////////////////////////////////////////////////////////
	//parameter transport
	CMenaceList m_MenaceList;
	int m_SampleNumber=2000;	
	for (int j=0;j<g_ThreatsCount;j++)
	{
		C_Menace temp;
		temp.Position_X=g_Threats[j].center.x;
		temp.Position_Y=g_Threats[j].center.y;
		temp.radius=g_Threats[j].range;
		m_MenaceList.push_back(temp);
	}
	POINT m_Start,m_End;
	m_Start.x=g_Sx;
	m_Start.y=g_Sy;
	m_End.x=g_Ex;
	m_End.y=g_Ey;
	if (g_ThreatsCount>100)
	{
		m_SampleNumber=10000;
	}else if (g_ThreatsCount<=100&&g_ThreatsCount>50)
	{
		m_SampleNumber=5000;
	}else{
		m_SampleNumber=1000;
	}

	
	if (prmMapVar_p!=NULL)
	{
		delete prmMapVar_p;
	}
	prmMapVar_p=new CPrmMap;
	prmMapVar_p->Init(g_MapWidth,g_MapHeight,m_SampleNumber,0,double(m_SampleNumber)/(g_MapWidth*g_MapHeight),m_Start, m_End, &m_MenaceList) ;

	if (!prmMapVar_p->GeneratePrm())
	{
		return false;
	}

	//////////////////////////////////////////////////////////////////////////
	//search initialize
	if (ACRoutePlannerVar_p!=NULL)
	{
		delete ACRoutePlannerVar_p;
	}
	ACRoutePlannerVar_p=new CACRoutePlanner;
	if (!ACRoutePlannerVar_p->InitAC(m_SampleNumber,prmMapVar_p->m_pVertex,20,1,2,1,0.1,0.1,m_Start,m_End,prmMapVar_p,0.7))
	{
		return false;
	}
	//////////////////////////////////////////////////////////////////////////	
	return true;
}

//search
bool Search()
{
	printf("PRM Planing...\n");
   	return ACRoutePlannerVar_p->RoutePlan();
}


void BuildSearchMap()
{
    
    if (g_SearchMap!=NULL)
    {
 	for (int i=0;i<g_MapHeight;i++)
        {
            delete g_SearchMap[i];
        }
        delete g_SearchMap;
	g_SearchMap=NULL;
    } 
    g_SearchMap=new int*[g_MapHeight];
    for (int i=0;i<g_MapHeight;i++)
    {
        g_SearchMap[i]=new int[g_MapWidth];
        memset(g_SearchMap[i],0,sizeof(int)*g_MapWidth);
    }	

    
    if (g_Terrain!=NULL)
    {
    	for (int i=0;i<g_MapHeight;i++)
   		for (int j=0;j<g_MapWidth;j++)
		{
			if (g_Terrain[i*g_MapWidth+j]>=g_FlyingHeight) 
				 g_SearchMap[i][j]=-1;
		}
    }
 
    
    for (int i=0;i<g_ThreatsCount;i++)
    {
        for (int ix=0;ix<g_Threats[i].range;ix++)
            for (int iy=0;iy<g_Threats[i].range;iy++)
            {
                if (ix*ix+iy*iy<g_Threats[i].range*g_Threats[i].range)
                {
                    g_SearchMap[g_Threats[i].center.y-iy][g_Threats[i].center.x-ix]=-1;
                    g_SearchMap[g_Threats[i].center.y+iy][g_Threats[i].center.x-ix]=-1;
                    g_SearchMap[g_Threats[i].center.y-iy][g_Threats[i].center.x+ix]=-1;
                    g_SearchMap[g_Threats[i].center.y+iy][g_Threats[i].center.x+ix]=-1;
                }
            }
    }
}


int GetRoad(POINT** Road)
{
	if (ACRoutePlannerVar_p->m_Route.size()<=0)
	{
		return -1;
	}

	long roadLen=(long)ACRoutePlannerVar_p->m_Route.size();
	*Road=new POINT[roadLen];
	POINT temp;
	
	CRoute::iterator itr=ACRoutePlannerVar_p->m_Route.begin();
	int idx=0;
	while( itr != ACRoutePlannerVar_p->m_Route.end() )
	{
		C_City TRout=*itr;
		temp.x=TRout.x;
		temp.y=TRout.y;
		(*Road)[idx]=temp;
		idx++;
		itr++;
	}
	return idx;
}

//-------------------------------------------------------------


