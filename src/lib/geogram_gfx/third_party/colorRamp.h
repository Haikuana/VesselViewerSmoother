#ifndef _COLOR_RAMP_
#define _COLOR_RAMP_

#include <cassert>
#include <math.h>

class CColorRamp
{
private :
	int m_Color[4][256];
	int m_Node[256];
	int  m_Size;
	int  m_NbNode;

public :

	// Constructor
	CColorRamp();
	~CColorRamp();

	// Data
	int GetSize() { return m_Size; }
	int Red(int index) { return m_Color[0][index]; }
	int Green(int index) { return m_Color[1][index]; }
	int Blue(int index) { return m_Color[2][index]; }

	// Misc
	int Build();
	void BuildDefault();
	int BuildNodes();
	void ResetNodes();

	void BuildRainbow();
	void BuildHsvmap();
	void BuildHotmap();
	void BuildSummer();

	void hsv2rgb(double* color ); //color[4]


};



	CColorRamp::CColorRamp()
	{
		BuildDefault();
	}

	//********************************************
	// Destructor
	//********************************************
	CColorRamp::~CColorRamp()
	{
	}

	//********************************************
	// Build
	// interpolate the colors between the nodes
	//********************************************
	int CColorRamp::Build()
	{
		int x1,y1,x2,y2;
		float a,b;
		assert(m_NbNode >= 2);
		for(int k=0;k<3;k++)
			for(int i=0;i<m_NbNode-1;i++)
			{
				x1 = (int)m_Node[i];
				x2 = (int)m_Node[i+1];


				assert(x1<x2);
				y1 = m_Color[k][x1];
				y2 = m_Color[k][x2];

				a = (float)(y2-y1) / (float)(x2-x1);
				b = (float)y1 - a*(float)x1;

				for(int j=x1;j<x2;j++)
					m_Color[k][j]=(int)(a*(float)j+b);
			}
			return 1;
	}

	//********************************************
	// Build
	//********************************************
	int CColorRamp::BuildNodes()
	{
		// Check first and last are set
		m_Color[3][0] = 1;
		m_Color[3][m_Size-1] = 1;
		// Count nodes
		m_NbNode = 0;
		for(int i=0;i<m_Size;i++)
			if(m_Color[3][i]==1)
			{
				m_Node[m_NbNode]=(int)i;
				m_NbNode++;
			}

			assert(m_NbNode>=2);
			return 1;
	}

	//********************************************
	// ResetNodes
	// Just first and last node
	//********************************************
	void CColorRamp::ResetNodes()
	{
		for(int i=0;i<m_Size;i++)
			m_Color[3][i] = 0;
		m_Color[3][0] = 1;
		m_Color[3][m_Size-1] = 1;
		BuildNodes();
	}

	//********************************************
	// BuildDefault
	// 256 grey levels
	//********************************************
	void CColorRamp::BuildDefault()
	{
		m_Size = 256;
		ResetNodes();

		// Grey scales
		for(int i=0;i<3;i++)
		{
			m_Color[i][0]=0;
			m_Color[i][255]=255;
		}
		Build();
	}

	//********************************************
	// BuildDefault
	// 256 grey levels
	//********************************************
	void CColorRamp::BuildRainbow()
	{
		m_Size = 256;
		ResetNodes();

		/*
		// Rainbow
		m_Color[3][0] = 1;m_Color[0][0] = 0;m_Color[1][0] = 0;m_Color[2][0] = 255;
		m_Color[3][48] = 1;m_Color[0][48] = 0;m_Color[1][48] = 254;m_Color[2][48] = 255;
		m_Color[3][96] = 1;m_Color[0][96] = 0;m_Color[1][96] = 254;m_Color[2][96] = 0;
		m_Color[3][144] = 1;m_Color[0][144] = 255;m_Color[1][144] = 255;m_Color[2][144] = 0;
		m_Color[3][192] = 1;m_Color[0][192] = 255;m_Color[1][192] = 126;m_Color[2][192] = 0;
		m_Color[3][240] = 1;m_Color[0][240] = 255;m_Color[1][240] = 0;m_Color[2][240] = 0;
		m_Color[3][255] = 1;m_Color[0][255] = 255;m_Color[1][255] = 255;m_Color[2][255] = 255;
		*/

		// Rainbow
		m_Color[3][0] = 1;m_Color[0][0] = 0;m_Color[1][0] = 0;m_Color[2][0] = 255;
		m_Color[3][48] = 1;m_Color[0][48] = 0;m_Color[1][48] = 254;m_Color[2][48] = 255;
		m_Color[3][96] = 1;m_Color[0][96] = 0;m_Color[1][96] = 254;m_Color[2][96] = 0;
		m_Color[3][144] = 1;m_Color[0][144] = 255;m_Color[1][144] = 255;m_Color[2][144] = 0;
		m_Color[3][192] = 1;m_Color[0][192] = 255;m_Color[1][192] = 126;m_Color[2][192] = 0;
		m_Color[3][240] = 1;m_Color[0][240] = 255;m_Color[1][240] = 20;m_Color[2][240] = 0;
		m_Color[3][255] = 1;m_Color[0][255] = 255;m_Color[1][255] = 0;m_Color[2][255] = 0;
		BuildNodes();
		Build();
	}
	//////////////////////////////////////////////////////////////////////////
	// convert the hsv to rgb
	// input:
	// color[0] -- h in [0, 360] 
	// color[1] -- s in [0, 1]
	// color[2] -- v in [0, 1]
	// color[3] -- alpha in [0, 1]
	// output: save the rgb still in color[4]
	void CColorRamp::hsv2rgb(double* color )
	{
		double H,S, V;
		H=color[0];
		S=color[1];
		V=color[2];
		double R,G, B;

		int i;
		double f, p, q, t;

		if( S == 0 ) 
		{
			// achromatic (grey)
			R = G = B = V;
			return;
		}

		H /= 60; // sector 0 to 5
		i = int(floor( H ));
		f = H - i; // factorial part of h
		p = V * ( 1 - S );
		q = V * ( 1 - S * f );
		t = V * ( 1 - S * ( 1 - f ) );

		switch( i ) 
		{
		case 0: 
			R = V;
			G = t;
			B = p;
			break;
		case 1:
			R = q;
			G = V;
			B = p;
			break;
		case 2:
			R = p;
			G = V;
			B = t;
			break;
		case 3:
			R = p;
			G = q;
			B = V;
			break;
		case 4:
			R = t;
			G = p;
			B = V;
			break;
		default: // case 5:
			R = V;
			G = p;
			B = q;
			break;
		}
		color[0]=R;
		color[1]=G;
		color[2]=B;
	}

	void CColorRamp::BuildHsvmap()
	{
		m_Size=256;
		double fStep=240/255.0;
		for (int i=0; i<m_Size; i++)
		{
			double color[4];
			color[0]=fStep*(255-i);
			color[1]=1.0;
			color[2]=0.7;
			color[3]=1.0;
			hsv2rgb(color);
			for (int k=0; k<4; k++)
			{
				m_Color[k][i]=255*color[k];
			}
		}
	}

	void CColorRamp::BuildHotmap()
	{
		m_Size=256;
		double fStep=256/3.0;
		for (int i=0; i<m_Size; i++)
		{
			if (i<fStep)
			{
				m_Color[0][i]=255*i/fStep;
				m_Color[1][i]=0;
				m_Color[2][i]=0;
				m_Color[3][i]=255;
			}
			else 	if ( i<2.0*fStep )
			{
				m_Color[0][i]=255;
				m_Color[1][i]=255*(i-fStep)/fStep;
				m_Color[2][i]=0;
				m_Color[3][i]=255;
			}	
			else
			{
				m_Color[0][i]=255;
				m_Color[1][i]=255;
				m_Color[2][i]=255*(i-2*fStep)/fStep;
				m_Color[3][i]=255;
			}	
		}
	}
	void CColorRamp::BuildSummer()
	{
		m_Size=256;
		double fStep=m_Size/3;
		//green to yellow
		for (int i=0; i<m_Size; i++)
		{
			if(i<fStep)

			{   m_Color[0][i]=255;
			m_Color[1][i]=128+i*128/fStep;
			m_Color[2][i]=0;
			m_Color[3][i]=255;
			}
			else
			{
				m_Color[0][i]=255;
				m_Color[1][i]=255;
				m_Color[2][i]=(i-fStep)*255/(2*fStep);
				m_Color[3][i]=255;
			}
		}

	}




#endif // _COLOR_RAMP_
