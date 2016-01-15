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
#include "hopfield_algo.h"

//-------------------------------------------------------------

//////////////////////////////////////////////////////////////////////////
//		{-1,-2},        { 1,-2},
// 	{-2,-1},{-1,-1},{ 0,-1},{ 1,-1},{ 2,-1},
//		{-1, 0},        { 1, 0},
//	{-2, 1},{-1, 1},{ 0, 1},{ 1, 1},{ 2, 1},
//		{-1, 2},        { 1, 2}
//////////////////////////////////////////////////////////////////////////
int DirAdd[16][2]={	{ 0,-1},
		{-1, 0},	{ 1, 0},
			{ 0, 1},
//////////////////////////////////////////////////////////////////////////
		{-1,-1},	{ 1,-1},

		{-1, 1},	{ 1, 1},
//////////////////////////////////////////////////////////////////////////
		{-1,-2},        { 1,-2},
	{-2,-1},			{ 2,-1},
	
	{-2, 1},			{ 2, 1},
		{-1, 2},        { 1, 2}
};
int nDirect=8;
//////////////////////////////////////////////////////////////////////////

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
int **g_DisMap=NULL;
double **g_Map=NULL;
POINT *g_Road=NULL;
POINT *g_Serial=NULL;
int g_nSerial=0;
int g_nStep=0;
int g_MaxRepNumber=2;
int g_A=10;
double g_InitValue=1e200;
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//get the threats and the configure parameters
bool InitThreat()
{
    char Temp[255];
    FILE* fp=fopen("src//micros_hopfield-master//map//Environment.ppf","r"); 
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
	FILE* fp = fopen("src//micros_hopfield-master//map//Terrain.raw","rb");
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

//get the search map
void BuildSearchMap()
{
    //initialize
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

void ConnectionSearch()
{
    int NowInput=0;
    int NowOutput=1;

    g_DisMap[g_Ey][g_Ex]=1;
    g_Serial[0].x=g_Ex;
    g_Serial[0].y=g_Ey;

    while (NowInput<NowOutput)
    {
        for (int i=0;i<nDirect;i++)
        {
            int X=g_Serial[NowInput].x+DirAdd[i][0];
            int Y=g_Serial[NowInput].y+DirAdd[i][1];
            if ((X>=0)&&(X<g_MapWidth)&&(Y>=0)&&(Y<g_MapHeight))
            {
                if (g_DisMap[Y][X]==0)
                {
                    g_Map[Y][X]=0.0f;
                    g_Serial[NowOutput].x=X;
                    g_Serial[NowOutput].y=Y;
                    g_DisMap[Y][X]=g_DisMap[g_Serial[NowInput].y][g_Serial[NowInput].x]+1;

                    NowOutput++;
                }
            }
        }
        NowInput++;
    }
    g_nSerial=NowOutput-1;
}

//Field
void FieldBuilding()
{
    for (int RepNumber=0;RepNumber<g_MaxRepNumber;RepNumber++)
    {
        int nowdis=1;
        int nowpos=0;

        int i=0;
        do {
            for (int j=0;j<g_A;j++)
            {
                for (i=nowpos;i<g_nSerial;i++)
                {
                    if (g_DisMap[g_Serial[i].y][g_Serial[i].x]>nowdis) break;
                    double Total=0;
                    if (i==0)
                    {
                        Total=g_InitValue;
                    }

                    for (int k=0;k<nDirect;k++)
                    {
                        int X=g_Serial[i].x+DirAdd[k][0];
                        int Y=g_Serial[i].y+DirAdd[k][1];
                        if ((X>=0)&&(X<g_MapWidth)&&(Y>=0)&&(Y<g_MapHeight))
                        {
                            if (g_DisMap[Y][X]>=0)
                            {
                                Total+=g_Map[Y][X];
                            }
                        }
                    }

                    Total/=g_A;
                    g_Map[g_Serial[i].y][g_Serial[i].x]=Total;
                }
            }
            nowdis++;
            nowpos=i;
        } while (nowpos<g_nSerial);
    }
}

//search
void RoadSearch()
{
    POINT m_Start;
    m_Start.x=g_Sx;
    m_Start.y=g_Sy;
    g_nStep=0;
    do
    {
        bool noget=true;
        double max=-1;
        POINT newstart={0,0};

        for (int k=0;k<nDirect;k++)
        {
            int X=m_Start.x+DirAdd[k][0];
            int Y=m_Start.y+DirAdd[k][1];
            if ((X>=0)&&(X<g_MapWidth)&&(Y>=0)&&(Y<g_MapHeight))
            {
                if (((g_Map[Y][X]>=max)||(noget))&&(g_DisMap[Y][X]>0))
                {
                    noget=false;
                    max=g_Map[Y][X];
                    newstart.x=X;
                    newstart.y=Y;
                }
            }
        }

        m_Start.x=newstart.x;
        m_Start.y=newstart.y;

        g_Road[g_nStep]=m_Start;

        g_nStep++;
    }while(((m_Start.x!=g_Ex)||(m_Start.y!=g_Ey))&&(g_nStep<g_MapWidth*g_MapHeight));
}
//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
//function for port

//clean the RAM
void Clean()
{
    if (g_Map!=NULL)
    {
        for (int i=0;i<g_MapHeight;i++)
        {
            delete g_Map[i];
            delete g_DisMap[i];
        }
        delete g_Map;
	g_Map=NULL;

        delete g_DisMap;
	g_DisMap=NULL;

        delete g_Serial;
	g_Serial=NULL;

        delete g_Road;
	g_Road=NULL;
    }
}

//initialize
bool InitData()
{
    Clean();

    g_nSerial=0;
    g_Serial=new POINT[g_MapWidth*g_MapHeight];
    g_Road=new POINT[g_MapWidth*g_MapHeight];
    g_DisMap=new int*[g_MapHeight];
    g_Map=new double*[g_MapHeight];

    for (int i=0;i<g_MapHeight;i++)
    {
        g_DisMap[i]=new int[g_MapWidth];
        g_Map[i]=new double[g_MapWidth];

        memcpy(g_DisMap[i],g_SearchMap[i],sizeof(int)*g_MapWidth);
    }

    return true;
}


bool Search()
{
    printf("Hopfield Planing...\n");
    ConnectionSearch();
    FieldBuilding();
    RoadSearch();
    return true;
}


int GetRoad(POINT** Road)
{
    *Road=new POINT[g_nStep];
    memcpy(*Road,g_Road,sizeof(POINT)*g_nStep);
    return g_nStep;
}

//-------------------------------------------------------------


