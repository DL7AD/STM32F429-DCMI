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

#ifndef __DCT_H__
#define __DCT_H__

#ifdef __cplusplus
extern "C" {
#endif

void dct_fill_tab(void);

// integer DCT 
void dct(conv pixel[8][8], conv data[8][8]);
void dct2(conv pixel[8][8], conv data[8][8]);
void dct3(conv pixel[8][8], conv data[8][8]);
void dct4(conv pixel[8][8], conv data[8][8]);
void dct5(conv pixel[8][8], conv data[8][8]);
void dct2_i(conv pixel[8][8], conv data[8][8]);
void dct2_s(conv pixel[8][8], conv data[8][8]);

// inverse real DCT
void idct(conv data[8][8], conv pixel[8][8]);
void idct2(conv data[8][8], conv pixel[8][8]);
void idct3(short data[8][8], short pixel[8][8]);
// inverse integer DCT 
void idct2_i(conv data[8][8], conv pixel[8][8]);
void idct2_s(conv data[8][8], conv pixel[8][8]);

//void test_idct2_s();

#ifdef __cplusplus
}
#endif

#endif//__DCT_H__
