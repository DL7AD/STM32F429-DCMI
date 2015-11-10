/******************************************************************************
** This file is part of the jpegant project.
**
** Copyright (C) 2009-2013 Vladimir Antonenko
**
** This program is free software; you can redistribute it and/or modify it
** under the terms of the GNU General Public License as published by the
** Free Software Foundation; either version 2 of the License,
** or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
** See the GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License along
** with this program; if not, write to the Free Software Foundation, Inc.
******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "arch.h"
#include "dct.h"

#define PI  3.1415926535897932384626433832795
#define IDCTI_AMP 512


uint64_t dctclk = 0;
uint64_t idctclk = 0;


// DCT basis functions coeficients
int dct_tbl_i[8][8];
float dct_tbl[8][8];

CACHE_ALIGN short dct_tbl_s[8][8] =
{
	{362, 362, 362, 362, 362, 362, 362, 362},
	{502, 425, 284,  99, -99,-284,-425,-502},
	{473, 195,-195,-473,-473,-195, 195, 473},
	{425, -99,-502,-284, 284, 502,  99,-425},
	{362,-362,-362, 362, 362,-362,-362, 362},
	{284,-502, 99,  425,-425, -99, 502,-284},
	{195,-473, 473,-195,-195, 473,-473, 195},
	{ 99,-284, 425,-502, 502,-425, 284, -99}
};

CACHE_ALIGN short idct_tbl_s[8][8] =
{
	{362, 502, 473, 425, 362, 284, 195,  99},
	{362, 425, 195, -99,-362,-502,-473,-284},
	{362, 284,-195,-502,-362,  99, 473, 425},
	{362,  99,-473,-284, 362, 425,-195,-502},
	{362, -99,-473, 284, 362,-425,-195, 502},
	{362,-284,-195, 502,-362, -99, 473,-425},
	{362,-425, 195,  99,-362, 502,-473, 284},
	{362,-502, 473,-425, 362,-284, 195, -99}
};

/******************************************************************************
**  dct
**  --------------------------------------------------------------------------
**  Fast DCT - Discrete Cosine Transform.
**  This function converts 8x8 pixel block into frequencies.
**  Lowest frequencies are at the upper-left corner.
**  The input and output could point at the same array, in this case the data
**  will be overwritten.
**  9 multiplications ans 33 additions for 1D DCT.
**  
**  ARGUMENTS:
**      pixels  - 8x8 pixel array;
**      data    - 8x8 freq block;
**
**  RETURN: -
******************************************************************************/
void dct(conv pixels[8][8], conv data[8][8])
{
	short rows[8][8];
	unsigned i;

	static const int
				c1 = 1004,  /* cos(pi/16) << 10 */
				s1 = 200,   /* sin(pi/16) */
				c3 = 851,   /* cos(3pi/16) << 10 */
				s3 = 569,   /* sin(3pi/16) << 10 */
				r2c6 = 554, /* sqrt(2)*cos(6pi/16) << 10 */
				r2s6 = 1337,/* sqrt(2)*sin(6pi/16) << 10 */
				r2 = 181;   /* sqrt(2) << 7*/

	/* transform rows */
	for (i = 0; i < 8; i++)
	{
		int x0,x1,x2,x3,x4,x5,x6,x7,x8;

		x0 = pixels[i][0];
		x1 = pixels[i][1];
		x2 = pixels[i][2];
		x3 = pixels[i][3];
		x4 = pixels[i][4];
		x5 = pixels[i][5];
		x6 = pixels[i][6];
		x7 = pixels[i][7];

		/* Stage 1 */
		x8=x7+x0;
		x0-=x7;
		x7=x1+x6;
		x1-=x6;
		x6=x2+x5;
		x2-=x5;
		x5=x3+x4;
		x3-=x4;

		/* Stage 2 */
		x4=x8+x5;
		x8-=x5;
		x5=x7+x6;
		x7-=x6;
		x6=c1*(x1+x2);
		x2=(-s1-c1)*x2+x6;
		x1=(s1-c1)*x1+x6;
		x6=c3*(x0+x3);
		x3=(-s3-c3)*x3+x6;
		x0=(s3-c3)*x0+x6;

		/* Stage 3 */
		x6=x4+x5;
		x4-=x5;
		x5=r2c6*(x7+x8);
		x7=(-r2s6-r2c6)*x7+x5;
		x8=(r2s6-r2c6)*x8+x5;
		x5=x0+x2;
		x0-=x2;
		x2=x3+x1;
		x3-=x1;

		/* Stage 4 and output */
		rows[i][0]=x6;
		rows[i][4]=x4;
		rows[i][2]=x8>>10;
		rows[i][6]=x7>>10;
		rows[i][7]=(x2-x5)>>10;
		rows[i][1]=(x2+x5)>>10;
		rows[i][3]=(x3*r2)>>17;
		rows[i][5]=(x0*r2)>>17;
	}

	/* transform columns */
	for (i = 0; i < 8; i++)
	{
		int x0,x1,x2,x3,x4,x5,x6,x7,x8;

		x0 = rows[0][i];
		x1 = rows[1][i];
		x2 = rows[2][i];
		x3 = rows[3][i];
		x4 = rows[4][i];
		x5 = rows[5][i];
		x6 = rows[6][i];
		x7 = rows[7][i];

		/* Stage 1 */
		x8=x7+x0;
		x0-=x7;
		x7=x1+x6;
		x1-=x6;
		x6=x2+x5;
		x2-=x5;
		x5=x3+x4;
		x3-=x4;

		/* Stage 2 */
		x4=x8+x5;
		x8-=x5;
		x5=x7+x6;
		x7-=x6;
		x6=c1*(x1+x2);
		x2=(-s1-c1)*x2+x6;
		x1=(s1-c1)*x1+x6;
		x6=c3*(x0+x3);
		x3=(-s3-c3)*x3+x6;
		x0=(s3-c3)*x0+x6;

		/* Stage 3 */
		x6=x4+x5;
		x4-=x5;
		x5=r2c6*(x7+x8);
		x7=(-r2s6-r2c6)*x7+x5;
		x8=(r2s6-r2c6)*x8+x5;
		x5=x0+x2;
		x0-=x2;
		x2=x3+x1;
		x3-=x1;

		/* Stage 4 and output */
		data[0][i]=((x6+16)>>3);
		data[4][i]=((x4+16)>>3);
		data[2][i]=((x8+16384)>>13);
		data[6][i]=((x7+16384)>>13);
		data[7][i]=((x2-x5+16384)>>13);
		data[1][i]=((x2+x5+16384)>>13);
		data[3][i]=(((x3>>8)*r2+8192)>>12);
		data[5][i]=(((x0>>8)*r2+8192)>>12);
	}
}
//
void dct_fill_tab()
{
	unsigned u,x;

	for (u = 0; u < 8; u++)
	{
		printf("%d: ", u);
		for (x = 0; x < 8; x++)
		{
			double Cu = (u==0)? 1.0/sqrt(2.0): 1.0;

			double K = Cu * cos((double)(2*x+1) * (double)u * PI/16.0);
			dct_tbl_i[u][x] = K * IDCTI_AMP;
			dct_tbl[u][x] = K;
			//dct_tbl_s[u][x] = K * IDCTI_AMP;
			//idct_tbl_s[x][u] = K * IDCTI_AMP; // different order

			printf("%f(%d),", K, idct_tbl_s[u][x]);
		}
		printf("\n");
	}
} 

/* real DCT
void dct2(conv pixel[8][8], conv data[8][8])
{
	unsigned x, y, n;
	float tmp[8][8];

//	uint64_t a = __rdtsc();

	for (y = 0; y < 8; y++)
	for (x = 0; x < 8; x++)
	{
		float q = 0.0f;

		for (n = 0; n < 8; n++)
			q += pixel[y][n] * dct_tbl[x][n];

		tmp[y][x] = q/2;
	}
		
	for (x = 0; x < 8; x++)
	for (y = 0; y < 8; y++)
	{
		float q = 0.0f;

		for (n = 0; n < 8; n++)
			q += tmp[n][x] * dct_tbl[y][n];

		data[y][x] = q/2;
	}

//	idctclk += __rdtsc() - a;
}*/

// integer DCT 
void dct2_i(conv pixel[8][8], conv data[8][8])
{
	unsigned x, y, n;
	conv tmp[8][8];

	//uint64_t a = __rdtsc();

	// process rows
	for (y = 0; y < 8; y++)
	for (x = 0; x < 8; x++)
	{
		int q = 0;

		for (n = 0; n < 8; n++)
			q += pixel[y][n] * dct_tbl_i[x][n];

		tmp[y][x] = (q + ((q<0)? -IDCTI_AMP: IDCTI_AMP))/(IDCTI_AMP*2);
	}
		
	// process columns
	for (x = 0; x < 8; x++)
	for (y = 0; y < 8; y++)
	{
		int q = 0;

		for (n = 0; n < 8; n++)
			q += tmp[n][x] * dct_tbl_i[y][n];

		data[y][x] = (q + ((q<0)? -IDCTI_AMP: IDCTI_AMP))/(IDCTI_AMP*2);
	}

	//idctclk += __rdtsc() - a;
} 

#ifdef _MSC_VER

void dct2_s(conv pixel[8][8], conv data[8][8])
{
	CACHE_ALIGN conv tmp[8][8];
	unsigned x, y;

	//uint64_t t = __rdtsc();

	// process rows
	for (y = 0; y < 8; y++) {

		__m128i a = _mm_loadu_si128 ((__m128i*)pixel[y]);

		for (x = 0; x < 8; x++) {
			__m128i b, c;

			b = _mm_load_si128 ((__m128i*)dct_tbl_s[x]);
			b = _mm_madd_epi16 (a, b);
			c = _mm_shuffle_epi32 (b, 0xB1);
			c = _mm_add_epi32 (b, c);
			b = _mm_shuffle_epi32 (c, 0x27);
			c = _mm_add_epi32 (b, c);
			tmp[x][y] = _mm_cvtsi128_si32(c)/(IDCTI_AMP*2);
		}
	}

	// process columns
	for (y = 0; y < 8; y++) {

		__m128i a = _mm_loadu_si128 ((__m128i*)tmp[y]);

		for (x = 0; x < 8; x++) {
			__m128i b, c;

			b = _mm_load_si128 ((__m128i*)dct_tbl_s[x]);
			b = _mm_madd_epi16 (a, b);
			c = _mm_shuffle_epi32 (b, 0xB1);
			c = _mm_add_epi32 (b, c);
			b = _mm_shuffle_epi32 (c, 0x27);
			c = _mm_add_epi32 (b, c);

			data[x][y] = _mm_cvtsi128_si32(c)/(IDCTI_AMP*2);
		}
	}

	//idctclk += __rdtsc() - t;
}

#endif//_MSC_VER


__inline static int sdiv(const int data, const int quant, const unsigned mag)
{
	(void)quant;
	return data >> mag;
	//return (data + quant) >> mag;
	//return (data + ((data<0)? -quant: quant))/(1<<mag);
}

// simple but fast DCT - 22 multiplication and 28 additions. 
void dct3(conv pixels[8][8], conv data[8][8])
{
	CACHE_ALIGN short rows[8][8];
	unsigned          i;

	static const short // Ci = cos(i*PI/16)*(1<<14);
		C1 = 16070,
		C2 = 15137,
		C3 = 13623,
		C4 = 11586,
		C5 = 9103,
		C6 = 6270,
		C7 = 3197;

	/* transform rows */
	for (i = 0; i < 8; i++)
	{
		short s07,s16,s25,s34,s0734,s1625;
		short d07,d16,d25,d34,d0734,d1625;

		s07 = pixels[i][0] + pixels[i][7];
		d07 = pixels[i][0] - pixels[i][7];
		s16 = pixels[i][1] + pixels[i][6];
		d16 = pixels[i][1] - pixels[i][6];
		s25 = pixels[i][2] + pixels[i][5];
		d25 = pixels[i][2] - pixels[i][5];
		s34 = pixels[i][3] + pixels[i][4];
		d34 = pixels[i][3] - pixels[i][4];

		rows[i][1] = (C1*d07 + C3*d16 + C5*d25 + C7*d34) >> 14;
		rows[i][3] = (C3*d07 - C7*d16 - C1*d25 - C5*d34) >> 14;
		rows[i][5] = (C5*d07 - C1*d16 + C7*d25 + C3*d34) >> 14;
		rows[i][7] = (C7*d07 - C5*d16 + C3*d25 - C1*d34) >> 14;

		s0734 = s07 + s34;
		d0734 = s07 - s34;
		s1625 = s16 + s25;
		d1625 = s16 - s25;

		rows[i][0] = (C4*(s0734 + s1625)) >> 14;
		rows[i][4] = (C4*(s0734 - s1625)) >> 14;

		rows[i][2] = (C2*d0734 + C6*d1625) >> 14;
		rows[i][6] = (C6*d0734 - C2*d1625) >> 14;
	}

	/* transform columns */
	for (i = 0; i < 8; i++)
	{
		short s07,s16,s25,s34,s0734,s1625;
		short d07,d16,d25,d34,d0734,d1625;

		s07 = rows[0][i] + rows[7][i];
		d07 = rows[0][i] - rows[7][i];
		s16 = rows[1][i] + rows[6][i];
		d16 = rows[1][i] - rows[6][i];
		s25 = rows[2][i] + rows[5][i];
		d25 = rows[2][i] - rows[5][i];
		s34 = rows[3][i] + rows[4][i];
		d34 = rows[3][i] - rows[4][i];

		data[1][i] = (C1*d07 + C3*d16 + C5*d25 + C7*d34) >> 16;
		data[3][i] = (C3*d07 - C7*d16 - C1*d25 - C5*d34) >> 16;
		data[5][i] = (C5*d07 - C1*d16 + C7*d25 + C3*d34) >> 16;
		data[7][i] = (C7*d07 - C5*d16 + C3*d25 - C1*d34) >> 16;

		s0734 = s07 + s34;
		d0734 = s07 - s34;
		s1625 = s16 + s25;
		d1625 = s16 - s25;

		data[0][i] = (C4*(s0734 + s1625)) >> 16;
		data[4][i] = (C4*(s0734 - s1625)) >> 16;

		data[2][i] = (C2*d0734 + C6*d1625) >> 16;
		data[6][i] = (C6*d0734 - C2*d1625) >> 16;
	}
}

// fast DCT, Vetterli & Ligtenberg - 16 multiplications and 26 additions.
void dct4(conv pixels[8][8], conv data[8][8])
{
	unsigned          i;
	CACHE_ALIGN short rows[8][8];

	static const short
		C1 = 16070,// cos(1*Pi/16) = 0.9808 * 16384
		S6 = 15137,// sin(6*Pi/16) = 0.9239
		C3 = 13623,// cos(3*Pi/16) = 0.8315
		C4 = 11586,// cos(4*Pi/16) = 0.7071
		S3 = 9102, // sin(3*Pi/16) = 0.5556
		C6 = 6270, // cos(6*Pi/16) = 0.3827
		S1 = 3196; // sin(1*Pi/16) = 0.1951

	/* transform rows */
	for (i = 0; i < 8; i++)
	{
		short s07,s12,s34,s56;
		short d07,d12,d34,d56;
		short x, y;

		s07 = pixels[i][0] + pixels[i][7];
		d07 = pixels[i][0] - pixels[i][7];
		s12 = pixels[i][1] + pixels[i][2];
		d12 = pixels[i][1] - pixels[i][2];
		s34 = pixels[i][3] + pixels[i][4];
		d34 = pixels[i][3] - pixels[i][4];
		s56 = pixels[i][5] + pixels[i][6];
		d56 = pixels[i][5] - pixels[i][6];

		x = s07 + s34;
		y = s12 + s56;
		rows[i][0] = C4*(x + y) >> 15;
		rows[i][4] = C4*(x - y) >> 15;

		x = d12 - d56;
		y = s07 - s34;
		rows[i][2] = (C6*x + S6*y) >> 15;
		rows[i][6] = (C6*y - S6*x) >> 15;

		x = d07 - (C4*(s12 - s56) >> 14);
		y = d34 - (C4*(d12 + d56) >> 14);
		rows[i][3] = (C3*x - S3*y) >> 15;
		rows[i][5] = (S3*x + C3*y) >> 15;

		x = (d07 << 1) - x;
		y = (d34 << 1) - y;
		rows[i][1] = (C1*x + S1*y) >> 15;
		rows[i][7] = (S1*x - C1*y) >> 15;
	}

	/* transform columns */
	for (i = 0; i < 8; i++)
	{
		short s07,s12,s34,s56;
		short d07,d12,d34,d56;
		short x, y;

		s07 = rows[0][i] + rows[7][i];
		d07 = rows[0][i] - rows[7][i];
		s12 = rows[1][i] + rows[2][i];
		d12 = rows[1][i] - rows[2][i];
		s34 = rows[3][i] + rows[4][i];
		d34 = rows[3][i] - rows[4][i];
		s56 = rows[5][i] + rows[6][i];
		d56 = rows[5][i] - rows[6][i];

		x = s07 + s34;
		y = s12 + s56;
		data[0][i] = C4*(x + y) >> 15;
		data[4][i] = C4*(x - y) >> 15;

		x = d12 - d56;
		y = s07 - s34;
		data[2][i] = (C6*x + S6*y) >> 15;
		data[6][i] = (C6*y - S6*x) >> 15;

		x = d07 - (C4*(s12 - s56) >> 14);
		y = d34 - (C4*(d12 + d56) >> 14);
		data[3][i] = (C3*x - S3*y) >> 15;
		data[5][i] = (S3*x + C3*y) >> 15;

		x = (d07 << 1) - x;
		y = (d34 << 1) - y;
		data[1][i] = (C1*x + S1*y) >> 15;
		data[7][i] = (S1*x - C1*y) >> 15;
	}
}

// simple but fast IDCT
void idct3(short data[8][8], short pixel[8][8])
{
	CACHE_ALIGN short rows[8][8];
	unsigned i;

	static const short // Ci = cos(i*PI/16)*(1<<14);
        C1 = 16070,
        C2 = 15137,
        C3 = 13623,
        C4 = 11586,
        C5 = 9103,
        C6 = 6270,
        C7 = 3196;

	/* transform rows */
	for (i = 0; i < 8; i++)
	{
		const short x0 = data[i][0];
		const short x4 = data[i][4];
		const short t0 = C4*(x0 + x4) >> 15;
		const short t4 = C4*(x0 - x4) >> 15;

		const short x2 = data[i][2];
		const short x6 = data[i][6];
		const short t2 = (C2*x2 + C6*x6) >> 15; 
		const short t6 = (C6*x2 - C2*x6) >> 15; 

		const short e0 = t0 + t2; 
		const short e3 = t0 - t2; 
		const short e1 = t4 + t6; 
		const short e2 = t4 - t6; 

		const short x1 = data[i][1];
		const short x3 = data[i][3];
		const short x5 = data[i][5];
		const short x7 = data[i][7];
		const short o0 = (C1*x1 + C5*x5 + C3*x3 + C7*x7) >> 15;
		const short o1 = (C3*x1 - C1*x5 - C7*x3 - C5*x7) >> 15;
		const short o2 = (C5*x1 + C7*x5 - C1*x3 + C3*x7) >> 15;
		const short o3 = (C7*x1 + C3*x5 - C5*x3 - C1*x7) >> 15;

		rows[i][0] = e0 + o0;
		rows[i][7] = e0 - o0;
		rows[i][1] = e1 + o1;
		rows[i][6] = e1 - o1;
		rows[i][2] = e2 + o2;
		rows[i][5] = e2 - o2;
		rows[i][3] = e3 + o3;
		rows[i][4] = e3 - o3;
	}

	/* transform columns */
	for (i = 0; i < 8; i++)
	{
		const short x0 = rows[0][i];
		const short x4 = rows[4][i];
		const short t0 = C4*(x0 + x4) >> 15;
		const short t4 = C4*(x0 - x4) >> 15;

		const short x2 = rows[2][i];
		const short x6 = rows[6][i];
		const short t2 = (C2*x2 + C6*x6) >> 15; 
		const short t6 = (C6*x2 - C2*x6) >> 15; 

		const short e0 = t0 + t2; 
		const short e3 = t0 - t2; 
		const short e1 = t4 + t6; 
		const short e2 = t4 - t6; 

		const short x1 = rows[1][i];
		const short x3 = rows[3][i];
		const short x5 = rows[5][i];
		const short x7 = rows[7][i];
		const short o0 = (C1*x1 + C5*x5 + C3*x3 + C7*x7) >> 15;
		const short o1 = (C3*x1 - C1*x5 - C7*x3 - C5*x7) >> 15;
		const short o2 = (C5*x1 + C7*x5 - C1*x3 + C3*x7) >> 15;
		const short o3 = (C7*x1 + C3*x5 - C5*x3 - C1*x7) >> 15;

		pixel[0][i] = e0 + o0;
		pixel[7][i] = e0 - o0;
		pixel[1][i] = e1 + o1;
		pixel[6][i] = e1 - o1;
		pixel[2][i] = e2 + o2;
		pixel[5][i] = e2 - o2;
		pixel[3][i] = e3 + o3;
		pixel[4][i] = e3 - o3;
	}
}
