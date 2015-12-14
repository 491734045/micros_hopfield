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
// PrmMap.cpp: implementation of the CPrmMap class.
//
//////////////////////////////////////////////////////////////////////

#include "math.h"
#include "PrmMap.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//##ModelId=4917DD9900BF
CPrmMap::CPrmMap()
{
	m_pVertex=NULL;
}

//##ModelId=4917DD9900C0
CPrmMap::~CPrmMap()
{
	if (m_pVertex!=NULL)
	{
		delete m_pVertex;
		m_pVertex=NULL;
	}
}

///////////////////////////////////////////////////////////////////////////////
// intilize the probability map

///////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD9900C2
void CPrmMap::Init(int width, int height, int vertex_num, int edge_num, double sample_pro, POINT start, POINT end, CMenaceList* pMenaceList)
{
	m_nWidth	= width ;
	m_nHeight	= height ;
	m_VertexNum = vertex_num ;
	m_EdgeNum	= edge_num ;
	m_SamplePro = sample_pro ;

	m_Start = start ;
	m_End = end ;

	m_pVertex = new Vertex[m_VertexNum] ;
	m_EdgeList.clear() ;

	m_pMenaceList = pMenaceList ;

	return ;
}


///////////////////////////////////////////////////////////////////////////////
//generate the probability map
///////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD9900ED
bool CPrmMap::GeneratePrm()
{
	int i ;

	// set the sample frequency
	m_SamplePro = double( m_nWidth * m_nHeight ) / double(m_VertexNum) ;

	
	m_pVertex[0].x = m_Start.x ;
	m_pVertex[0].y = m_Start.y ;
	m_pVertex[1].x = m_End.x ;
	m_pVertex[1].y = m_End.y ;

	
	int nCount = 2 ;
	int x, y ;
	while( nCount < m_VertexNum )
	{
		
		x = random( 0, m_nWidth-1 ) ;
		y = random( 0, m_nHeight-1 ) ;
		m_pVertex[nCount].x = x ;
		m_pVertex[nCount].y = y ;

	
		for( i=0;i<nCount;i++ )
		{
			bool bConnected = false ;
			bConnected = LocalPlanner(m_pVertex[nCount], m_pVertex[i], 10, 100) ;
			if( bConnected == true )
			{
				
				Edge edge ;
				edge.vertex_index1 = nCount ;
				edge.vertex_index2 = i ;
				m_EdgeList.push_back(edge) ;
			}
			else
			{
				
			}
		}

		nCount ++ ;
	}

	return true ;
}

///////////////////////////////////////////////////////////////////////////////
// generate the random in (min,max)

///////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD9901E7
int CPrmMap::random(int min, int max)
{
	int num = 0 ;
	if( max <= min )
	{
		return -1 ;
	}
	num = ( rand() % (max - min) ) + min ;
	return num ;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD9901D5
bool CPrmMap::LocalPlanner(Vertex vertex1, Vertex vertex2, double Dmin, double Dmax)
{
	double step = 10 ;
	double dis ;
	dis = sqrt((double)
				((vertex1.x - vertex2.x)*(vertex1.x - vertex2.x) +
		        (vertex1.y - vertex2.y)*(vertex1.y - vertex2.y) ) );
	if( (dis < Dmin) || (dis > Dmax) )
	{
		return false ;
	}

	
	C_Menace menace ;
	double x,y ;
	double dtemp ;
	int count = int(dis / step) ;
	int i ;
	for( i=0;i<count;i++ )
	{
		x = vertex1.x + ( vertex2.x - vertex1.x )*( (step*i)/dis ) ;
		y = vertex1.y + ( vertex2.y - vertex1.y )*( (step*i)/dis ) ;
		
		
		if ((x<0)||(x>=g_MapWidth)||(y<0)||(y>=g_MapHeight)) continue;

		if (g_SearchMap[int(y)][int(x)]<0) return false;

		/*CMenaceList::iterator itr=m_pMenaceList->begin();
		int idx=0;
		while( itr != m_pMenaceList->end() )
		{
			menace = *itr;

			dtemp = Distance( x, y, double(menace.Position_X), double(menace.Position_Y) ) ;
			if( dtemp < menace.radius )
			{
				return false ;
			}

			itr++;
		}*/
	}
	
	return true ;
}


//##ModelId=4917DD9900DF
double CPrmMap::Distance(double x1, double y1, double x2, double y2)
{
	double dis ;
	dis = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)) ;
	return dis ;
}
