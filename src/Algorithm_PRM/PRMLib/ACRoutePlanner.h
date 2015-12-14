
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
// ACRoutePlanner.h: interface for the CACRoutePlanner class.
//
//////////////////////////////////////////////////////////////////////
#ifndef ACROUTEPLANNER_H_
#define ACROUTEPLANNER_H_
#include "PrmMap.h"


//##ModelId=4917DD99033C
class CACRoutePlanner  
{
public:
	//##ModelId=4917DD99033D
	CACRoutePlanner();
	//##ModelId=4917DD99034B
	virtual ~CACRoutePlanner();

public:
	//##ModelId=4917DD99034D
	bool GlobalUpdate();
	//##ModelId=4917DD99035B
	bool LocalUpdate(C_Ant* ant);
	//##ModelId=4917DD99035D
	bool RouteFinish(C_Ant* ant);
	//##ModelId=4917DD99035F
	int SelectNextCity(C_Ant* ant);
	//##ModelId=4917DD99036B
	bool RoutePlan();
	//##ModelId=4917DD99036C
	bool ACRoutePlanner();
	//##ModelId=4917DD99036D
	double Distance(int x1, int y1, int x2, int y2);
	//##ModelId=4917DD99037E
	bool InitAC(int CityNum, Vertex* pVertex, int AntNum, double Alpha, double Beta, double PheromoneUnit, double RoGlobal, double RoLocal, POINT start, POINT end, CPrmMap* prm, double Q0);

	//##ModelId=4917DD99039D
	int			m_CityNum ;
	//##ModelId=4917DD9903AA
	C_City *	m_pCity ;
	//##ModelId=4917DD9903AE
	double **	m_ppCityDis ;


	
	//##ModelId=4917DD9903B9
	bool**		m_ppCityConnect ;		
	//##ModelId=4917DD9903BA
	int			m_AntNum ;				
	//##ModelId=4917DD9903BB
	double		m_Alpha ;				
	//##ModelId=4917DD9903C8
	double		m_Beta ;				
	//##ModelId=4917DD9903C9
	double		m_PheromoneUnit ;		
	//##ModelId=4917DD9903CA
	double		m_RoGlobal ;			
	//##ModelId=4917DD9903D8
	double		m_RoLocal ;				
	//##ModelId=4917DD9903D9
	double**	m_ppPheromone ;			
	//##ModelId=4917DD9903DA
	double**	m_ppEta ;				
	//##ModelId=4917DD9A0000
	int			m_nCount ;				
	//##ModelId=4917DD9A0001
	double		m_Q0 ;					
	//##ModelId=4917DD9A000F
	double		m_Lnn ;


	
	//##ModelId=4917DD9A0011
	C_Ant*		m_pAnt ;				

	
	//##ModelId=4917DD9A0015
	POINT		m_Start ;				
	//##ModelId=4917DD9A001F
	int			m_StartIndex ;			
	//##ModelId=4917DD9A0020
	POINT		m_End ;					
	//##ModelId=4917DD9A002E
	int			m_EndIndex ;			
	//##ModelId=4917DD9A0030
	CRoute		m_Route ;				
	//##ModelId=4917DD9A003E
	double		m_Fbest ;				

	
	//##ModelId=4917DD9A003F
	double m_Pij[1000] ;
	//##ModelId=4917DD9A0040
	int m_ChildIndex[1000] ;
	//##ModelId=4917DD9A004E
	bool m_bVertexPassed[1000] ;


	//##ModelId=4917DD9A004F
	double temp ;
};

#endif
