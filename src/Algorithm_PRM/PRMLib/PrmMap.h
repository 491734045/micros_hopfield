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
// PrmMap.h: interface for the CPrmMap class.
//
//////////////////////////////////////////////////////////////////////

#ifndef PRMMAP_H_
#define PRMMAP_H_
#include "DataStruct.h"
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//##ModelId=4917DD9900BE
class CPrmMap  
{
public:
	//##ModelId=4917DD9900BF
	CPrmMap();
	//##ModelId=4917DD9900C0
	//jade+ << 08.11.13
	
	virtual ~CPrmMap();	
	//##ModelId=4917DD9900C2
	void Init(int width, int height, int vertex_num, int edge_num, double sample_pro, POINT start, POINT end, CMenaceList* pMenaceList);

public:
	//##ModelId=4917DD9900DF
	double Distance(double x1, double y1, double x2, double y2);
	//##ModelId=4917DD9900ED
	bool GeneratePrm();

	//##ModelId=4917DD990197
	CMenaceList*	m_pMenaceList ;				

	//##ModelId=4917DD99019B
	double		m_SamplePro		;				

	//##ModelId=4917DD99019C
	int			m_nWidth		;				
	//##ModelId=4917DD9901A5
	int			m_nHeight		;				

	//##ModelId=4917DD9901B5
	int			m_VertexNum		;				
	//##ModelId=4917DD9901B6
	int			m_EdgeNum		;				

	//##ModelId=4917DD9901B8
	Vertex *	m_pVertex		;				
	//##ModelId=4917DD9901C6
	CEdgeList   m_EdgeList		;				

	//##ModelId=4917DD9901CA
	POINT		m_Start			;				
	//##ModelId=4917DD9901D4
	POINT		m_End			;				

	
private:
	//##ModelId=4917DD9901D5
	bool LocalPlanner(Vertex vertex1, Vertex vertex2, double Dmin, double Dmax);
	//##ModelId=4917DD9901E7
	int random(int min, int max);

protected:

};

#endif //
