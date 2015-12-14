
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
// ACRoutePlanner.cpp: implementation of the CACRoutePlanner class.
//
//////////////////////////////////////////////////////////////////////

#include "math.h"
#include "ACRoutePlanner.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//##ModelId=4917DD99033D
CACRoutePlanner::CACRoutePlanner()
{
	m_pCity=NULL;
	m_ppCityDis=NULL;
	m_ppCityConnect=NULL; 
	m_ppPheromone=NULL;
	m_ppEta=NULL;
	m_pAnt=NULL;
	m_CityNum=0;

}


//##ModelId=4917DD99034B
CACRoutePlanner::~CACRoutePlanner()
{

	if (m_pCity!=NULL)
	{
		delete []m_pCity;
		m_pCity=NULL;
	}
	if (m_ppCityDis!=NULL)
	{
		for (int i=0;i<m_CityNum;i++)
		{
			delete m_ppCityDis[i];
		}
		delete m_ppCityDis;
		m_ppCityDis=NULL;
	}
	if (m_ppCityConnect!=NULL)
	{
		for (int i=0;i<m_CityNum;i++)
		{
			delete m_ppCityConnect[i];
		}
		delete m_ppCityConnect;
		m_ppCityConnect=NULL;
	}
	if (m_ppPheromone!=NULL)
	{
		for (int i=0;i<m_CityNum;i++)
		{
			delete m_ppPheromone[i];
		}
		delete m_ppPheromone;
		m_ppPheromone=NULL;
	}
	if (m_ppEta!=NULL)
	{
		for (int i=0;i<m_CityNum;i++)
		{
			delete m_ppEta[i];
		}
		delete m_ppEta;
		m_ppEta=NULL;
	}
	if (m_pAnt!=NULL)
	{
		delete []m_pAnt;
		m_pAnt=NULL;
	}
}


/////////////////////////////////////////////////////////////////////////////////
// initilize the ant colony optimization,ACO
/////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99037E
bool CACRoutePlanner::InitAC(int CityNum, Vertex *pVertex, int AntNum, double Alpha, double Beta, double PheromoneUnit, double RoGlobal, double RoLocal, POINT start, POINT end, CPrmMap *prm, double Q0)
{
	int i,j ;
	if (m_pCity!=NULL)
	{
		delete m_pCity;
		m_pCity=NULL;
	}
	this->m_CityNum = CityNum ;
	m_pCity = new C_City[m_CityNum] ;
	for( i=0;i<m_CityNum;i++ )
	{
		m_pCity[i].index = i ;
		m_pCity[i].x = pVertex[i].x ;
		m_pCity[i].y = pVertex[i].y ;
		m_pCity[i].longitude = pVertex[i].longitude ;
		m_pCity[i].latitude = pVertex[i].latitude ;
		m_pCity[i].altitude = pVertex[i].altitude ;
	}

	m_AntNum = AntNum ;
	m_pAnt = new C_Ant[m_AntNum] ;
	m_Alpha = Alpha ;
	m_Beta = Beta ;
	m_PheromoneUnit = PheromoneUnit ;
	m_RoGlobal = RoGlobal ;
	m_RoLocal = RoLocal ;
	m_Start = start ;
	m_End = end ;

	m_ppPheromone = new double*[m_CityNum] ;
	m_ppCityDis = new double*[m_CityNum] ;
	m_ppEta = new double*[m_CityNum] ;
	m_ppCityConnect = new bool*[m_CityNum] ;
	for( i=0;i<m_CityNum;i++ )
	{
		m_ppPheromone[i] = new double[m_CityNum] ;
		m_ppCityDis[i] = new double[m_CityNum] ;
		m_ppEta[i] = new double[m_CityNum] ;
		m_ppCityConnect[i] = new bool[m_CityNum] ;
		for( j=0;j<m_CityNum;j++ )
		{
			m_ppPheromone[i][j] = 1 / (m_CityNum*1500.0) ;
			m_ppCityDis[i][j] = Distance(m_pCity[i].x,m_pCity[i].y,m_pCity[j].x,m_pCity[j].y) ;
			m_ppEta[i][j] = 1 / m_ppCityDis[i][j] ;
			m_ppCityConnect[i][j] = false ;
		}
	}

	CEdgeList::iterator itr=prm->m_EdgeList.begin();
	int idx=0;
	Edge edge ;
	while( itr != prm->m_EdgeList.end() )
	{
			
		edge = *itr ;
		m_ppCityConnect[edge.vertex_index1][edge.vertex_index2] = true ;
		m_ppCityConnect[edge.vertex_index2][edge.vertex_index1] = true ;
		itr++;
	}


	m_Start = start ;
	m_End = end ;
	m_StartIndex = 0 ;
	m_EndIndex = 1 ;

	m_Fbest = 100000000 ;
	m_Route.clear() ;

	m_Q0 = Q0 ;
	m_Lnn = 2500 ;

	return true ;
}

//##ModelId=4917DD99036D
double CACRoutePlanner::Distance(int x1, int y1, int x2, int y2)
{
	double dis ;
	dis = sqrt((double)
		(( x1 - x2 )*( x1 - x2 ) + ( y1 - y2 )*( y1 - y2 )) );
	return dis ;
}

/////////////////////////////////////////////////////////////////////////////////
//plan using the ant colony optimization,ACO
/////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99036C
bool CACRoutePlanner::ACRoutePlanner()
{
	int count = 1000 ;
	int stable_time = 0 ;
	double Fprev = 0 ;
	for( int t=0;t<count;t++ )
	{
		RoutePlan() ;
		if( (fabs(m_Fbest - Fprev))<0.000001 )
		{
			stable_time ++ ;
			if( stable_time > 100 )
				return true ;
		}
		else
		{
			stable_time = 0 ;
		}
		Fprev = m_Fbest ;
	}
	return true ;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99036B
bool CACRoutePlanner::RoutePlan()
{
	int i ;
	int AntIndex ;

	// initilize the ACO
	for( AntIndex=0;AntIndex<m_AntNum;AntIndex++ )
	{
		m_pAnt[AntIndex].AntIndex = AntIndex ;
		m_pAnt[AntIndex].VertexIndex = m_StartIndex ;
		m_pAnt[AntIndex].Position_X = m_Start.x ;
		m_pAnt[AntIndex].Position_Y = m_Start.y ;
		m_pAnt[AntIndex].AtTime = 0 ;
		m_pAnt[AntIndex].ErstTime = 0 ;
		m_pAnt[AntIndex].LastTime = 0 ;
		m_pAnt[AntIndex].Route.clear() ;
		m_pAnt[AntIndex].Route.push_back(m_pCity[m_StartIndex]) ;
		m_pAnt[AntIndex].Length = 0 ;
		m_pAnt[AntIndex].Menace = 0 ;
		m_pAnt[AntIndex].F = 0 ;
	}

	
	for( AntIndex=0;AntIndex<m_AntNum;AntIndex++)
	{
		for( i=0;i<m_CityNum;i++ )
		{
			m_bVertexPassed[i] = false ;
		}
		m_bVertexPassed[m_StartIndex] = true ;

		int NextCityIndex ;
		while( 1 )
		{
			
			NextCityIndex = SelectNextCity(&m_pAnt[AntIndex]) ;

			if( NextCityIndex == -1 )
			{
				
				m_pAnt[AntIndex].Length = 10000000000 ;
				m_pAnt[AntIndex].F = 10000000000 ;
				break ;
			}

			C_City city_tail ;
			city_tail = m_pAnt[AntIndex].Route.back() ;

			
			m_pAnt[AntIndex].Route.push_back(m_pCity[NextCityIndex]) ;
			m_pAnt[AntIndex].VertexIndex = NextCityIndex ;
			m_pAnt[AntIndex].Position_X = m_pCity[NextCityIndex].x ;
			m_pAnt[AntIndex].Position_Y = m_pCity[NextCityIndex].y ;
			m_bVertexPassed[NextCityIndex] = true ;
			
			m_pAnt[AntIndex].Length += Distance(city_tail.x,city_tail.y,m_pCity[NextCityIndex].x,m_pCity[NextCityIndex].y) ;

			
			if( RouteFinish(&m_pAnt[AntIndex]) == true )
			{
				
				m_pAnt[AntIndex].F = m_pAnt[AntIndex].Length ;
				break ;
			}
			else
			{
				
			}
		}

		
		LocalUpdate(&m_pAnt[AntIndex]) ;
	}

	
	GlobalUpdate() ;


	
	double F_min = 10000000000 ;
	int index_best = -1 ;
	for( AntIndex=0;AntIndex<m_AntNum;AntIndex++ )
	{
		if( m_pAnt[AntIndex].F > 100000000 )
		{
			continue ;
		}
		if( m_pAnt[AntIndex].F < F_min )
		{
			F_min = m_pAnt[AntIndex].F ;
			index_best = AntIndex ;
		}
	}
	if( index_best != -1 )
	{
		
		if( m_pAnt[index_best].F < m_Fbest )
		{
			C_City city ;
			m_Fbest = m_pAnt[index_best].F ;
			m_Route.clear() ;
			

			CRoute::iterator itr= m_pAnt[index_best].Route.begin();
			int idx=0;
			while( itr !=  m_pAnt[index_best].Route.end() )
			{
				city = *itr;
				m_Route.push_back(city) ;
				itr++;
			}
		}

		temp = m_pAnt[index_best].F ;
	}

	return true ;
}


/////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99035F
int CACRoutePlanner::SelectNextCity(C_Ant *ant)
{
	int NextCityIndex ;
	double Q ;
	double Eta ;

	Q = rand() / (double)RAND_MAX ;
	if( Q < m_Q0 )
	{
		
		double s = 0 ;
		double s_max = 0 ;
		int index_max = -1 ;
		for(int index=0;index<m_CityNum;index++ )
		{
			if( index == ant->VertexIndex )
			{
				continue ;
			}
			if( m_bVertexPassed[index] == true )
			{
				continue ;
			}
			if( (m_ppCityConnect[index][ant->VertexIndex]==false) || (m_ppCityConnect[ant->VertexIndex][index]==false) )
			{
				continue ;
			}
			Eta = 1 / Distance(m_pCity[index].x,m_pCity[index].y,m_pCity[m_EndIndex].x,m_pCity[m_EndIndex].y) ;
			if( (m_ppCityConnect[index][m_EndIndex]==true) || (m_ppCityConnect[m_EndIndex][index]==true) )
			{
				Eta += 1 ;
			}
			s = pow(m_ppPheromone[ant->VertexIndex][index],m_Alpha)*pow(Eta,m_Beta) ;
			if( s > s_max )
			{
				s_max = s ;
				index_max = index ;
			}
		}
		NextCityIndex = index_max ;
	}
	else
	{
		
		for(int i=0;i<1000;i++)
		{
			m_Pij[i] = 0 ;
			m_ChildIndex[i] = -1 ;
		}

		
		int nCount = 0 ;
		double sum = 0 ;
		for(int index=0;index<m_CityNum;index++ )
		{
			if( index == ant->VertexIndex )
			{
				continue ;
			}
			if( m_bVertexPassed[index] == true )
			{
				continue ;
			}
			if( (m_ppCityConnect[index][ant->VertexIndex]==false) || (m_ppCityConnect[ant->VertexIndex][index]==false) )
			{
				continue ;
			}
			Eta = 1 / Distance(m_pCity[index].x,m_pCity[index].y,m_pCity[m_EndIndex].x,m_pCity[m_EndIndex].y) ;
			if( (m_ppCityConnect[index][m_EndIndex]==true) || (m_ppCityConnect[m_EndIndex][index]==true) )
			{
				Eta += 1 ;
			}
			m_Pij[index] = pow(m_ppPheromone[ant->VertexIndex][index],m_Alpha)*pow(Eta,m_Beta) ;
			sum += m_Pij[index] ;
			
			m_ChildIndex[nCount] = index ;
			nCount ++ ;
		}

		
		for(int i=0;i<nCount;i++ )
		{
			m_Pij[ m_ChildIndex[i] ] = m_Pij[ m_ChildIndex[i] ] / sum ;
		}

		
		if( nCount == 0 )
		{
			return -1 ;
		}

		
		double r ;
		r = rand() / (double)RAND_MAX ;
		for(int i=0;i<nCount;i++ )
		{
			r = r - m_Pij[ m_ChildIndex[i] ] ;
			if( r <= 0 )
			{
				NextCityIndex = m_ChildIndex[i] ;
				return NextCityIndex ;
			}
		}
		NextCityIndex = m_ChildIndex[nCount-1] ;
		return NextCityIndex ;
	}

	return NextCityIndex ;
}


///////////////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99035D
bool CACRoutePlanner::RouteFinish(C_Ant *ant)
{
	C_City city ;
	city = ant->Route.back() ;
	if( city.index == m_EndIndex )
	{
		return true ;
	}
	else
	{
		return false ;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99035B
bool CACRoutePlanner::LocalUpdate(C_Ant *ant)
{
	if( ant->Route.size() == 0 )
	{
		printf("WRONG FOR UPDATING!\n") ;
		return false ;
	}

	CRoute::iterator itr1;
	CRoute::iterator itr2;	
	
	C_City city1,city2 ;

	if( ant->F > 100000000 )
	{
	
		itr1= ant->Route.begin();
		itr2= itr1;
		itr2++;	
	
		while( itr2 != ant->Route.end() )
		{
			city1 = *itr1 ;
			city2 = *itr2 ;

			m_ppPheromone[city1.index][city2.index] = (1-m_RoLocal)*m_ppPheromone[city1.index][city2.index] ;
			m_ppPheromone[city2.index][city1.index] = m_ppPheromone[city1.index][city2.index] ;

			itr1++;
			itr2++;
		}
	}
	else
	{
		
		itr1= ant->Route.begin();
		itr2= itr1;
		itr2++;	

		while( itr2 != ant->Route.end() )
		{
			city1 = *itr1 ;
			city2 = *itr2 ;
			m_ppPheromone[city1.index][city2.index] = (1-m_RoLocal)*m_ppPheromone[city1.index][city2.index] ;
			m_ppPheromone[city1.index][city2.index] += m_RoLocal*( m_PheromoneUnit / (m_CityNum*m_Lnn) ) ;
			m_ppPheromone[city2.index][city1.index] = m_ppPheromone[city1.index][city2.index] ;
			itr1++;
			itr2++;
		}
	}
	return true ;
}


///////////////////////////////////////////////////////////////////////////////////////////
//##ModelId=4917DD99034D
bool CACRoutePlanner::GlobalUpdate()
{
	int index ;
	C_City city1,city2 ;

	
	double F_min = 100000000 ;
	int index_best = -1 ;
	for( index=0;index<m_AntNum;index++ )
	{
		if( m_pAnt[index].F > 100000000 )
		{
			continue ;
		}

		if( m_pAnt[index].F < F_min )
		{
			F_min = m_pAnt[index].F ;
			index_best = index ;
		}
	}

	CRoute::iterator itr1;
	CRoute::iterator itr2;	

	if( index_best == -1 )
	{
		
		for( index=0;index<m_AntNum;index++ )
		{
			itr1= m_pAnt[index].Route.begin();
			itr2= itr1;
			itr2++;	

			while(itr2 != m_pAnt[index].Route.end() )
			{
				city1 = *itr1;
				city2 = *itr2;
				m_ppPheromone[city1.index][city2.index] = (1-m_RoGlobal)*m_ppPheromone[city1.index][city2.index] ;
				m_ppPheromone[city2.index][city1.index] = m_ppPheromone[city1.index][city2.index] ;
				itr1++;
				itr2++;
			}
		}
	}
	else
	{
		if( m_pAnt[index_best].Route.size() == 0 )
		{
			printf("全局信息素更新错误!\n") ;
			return false ;
		}

		
		int i,j ;
		for( i=0;i<m_CityNum;i++ )
		{
			for( j=0;j<m_CityNum;j++ )
			{
				m_ppPheromone[i][j] = (1-m_RoGlobal)*m_ppPheromone[i][j] ;
			}
		}

		itr1= m_pAnt[index_best].Route.begin();
		itr2= itr1;
		itr2++;	

		while( itr2 !=  m_pAnt[index_best].Route.end() )
		{
			city1 = *itr1;
			city2 = *itr2;
			m_ppPheromone[city1.index][city2.index] += m_RoGlobal*(1 / m_pAnt[index_best].Length) ;
			m_ppPheromone[city2.index][city1.index] = m_ppPheromone[city1.index][city2.index] ;
			itr1++;
			itr2++;
		}
	}

	return true ;
}
