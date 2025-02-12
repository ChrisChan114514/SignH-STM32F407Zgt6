// 
///*预定义汉字点阵数据文件*/
//
///*------------------------------------------------------------------------------
#ifndef __GB1616_H
#define __GB1616_H

#include "fonts.h"


/////////////////////////////////////////////////////////////////////////
// 汉字字模表                                                          //
// 取模软件：   纵向取模 -> PCtoLCD2002完美版：逐行式                         //
/////////////////////////////////////////////////////////////////////////
const struct  typFNT_GBxx codeGB_16[] =          // 数据表 
{

{"校",  0x10, 0x40, 0x10, 0x20, 0x10, 0x20, 0x11, 0xFE, 0xFC, 0x00, 0x10, 0x88, 0x31, 0x04, 0x3A, 0x02, 
        0x54, 0x88, 0x50, 0x88, 0x90, 0x50, 0x10, 0x50, 0x10, 0x20, 0x10, 0x50, 0x10, 0x88, 0x13, 0x06 }, /*"校",1*/

{"准",  0x01, 0x40, 0x41, 0x20, 0x21, 0x20, 0x23, 0xFE, 0x02, 0x20, 0x16, 0x20, 0x1B, 0xFC, 0x12, 0x20, 
        0x22, 0x20, 0x23, 0xFC, 0xE2, 0x20, 0x22, 0x20, 0x22, 0x20, 0x23, 0xFE, 0x22, 0x00, 0x02, 0x00 }, /*"准",2*/

{"幅",  0x10, 0x00, 0x11, 0xFE, 0x10, 0x00, 0x7C, 0xF8, 0x54, 0x88, 0x54, 0x88, 0x54, 0xF8, 0x54, 0x00, 
        0x55, 0xFC, 0x55, 0x24, 0x55, 0x24, 0x5D, 0xFC, 0x11, 0x24, 0x11, 0x24, 0x11, 0xFC, 0x11, 0x04 }, /*"幅",3*/

{"值",  0x08, 0x40, 0x08, 0x40, 0x0F, 0xFC, 0x10, 0x40, 0x10, 0x40, 0x33, 0xF8, 0x32, 0x08, 0x53, 0xF8, 
        0x92, 0x08, 0x13, 0xF8, 0x12, 0x08, 0x13, 0xF8, 0x12, 0x08, 0x12, 0x08, 0x1F, 0xFE, 0x10, 0x00 }, /*"值",4*/

{"占",  0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0xFE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 
        0x3F, 0xF8, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x3F, 0xF8, 0x20, 0x08 }, /*"占",5*/

{"空",  0x02, 0x00, 0x01, 0x00, 0x7F, 0xFE, 0x40, 0x02, 0x88, 0x24, 0x10, 0x10, 0x20, 0x08, 0x00, 0x00, 
        0x1F, 0xF0, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7F, 0xFC, 0x00, 0x00 }, /*"空",6*/

{"比",  0x00, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x84, 0x20, 0x88, 0x20, 0x90, 0x3E, 0xA0, 0x20, 0xC0, 
        0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x82, 0x26, 0x82, 0x38, 0x82, 0x20, 0x7E, 0x00, 0x00 }, /*"比",7*/

{"频",  0x10, 0x00, 0x11, 0xFE, 0x50, 0x20, 0x5C, 0x40, 0x51, 0xFC, 0x51, 0x04, 0xFF, 0x24, 0x01, 0x24, 
        0x11, 0x24, 0x55, 0x24, 0x55, 0x24, 0x55, 0x44, 0x84, 0x50, 0x08, 0x88, 0x31, 0x04, 0xC2, 0x02 }, /*"频",8*/

{"率",  0x02, 0x00, 0x01, 0x00, 0x7F, 0xFC, 0x02, 0x00, 0x44, 0x44, 0x2F, 0x88, 0x11, 0x10, 0x22, 0x48, 
        0x4F, 0xE4, 0x00, 0x20, 0x01, 0x00, 0xFF, 0xFE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00 }, /*"率",9*/

{"凌",  0x00, 0x40, 0x40, 0x40, 0x23, 0xF8, 0x20, 0x40, 0x00, 0x40, 0x07, 0xFE, 0x11, 0x10, 0x12, 0x88, 
        0x24, 0x84, 0xE1, 0xF0, 0x23, 0x10, 0x24, 0xA0, 0x20, 0x40, 0x20, 0xA0, 0x23, 0x10, 0x0C, 0x0C }, /*"凌",10*/

{"智",  0x20, 0x00, 0x3E, 0x7C, 0x48, 0x44, 0x08, 0x44, 0xFF, 0x44, 0x14, 0x44, 0x22, 0x7C, 0x40, 0x00, 
        0x1F, 0xF0, 0x10, 0x10, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10 }, /*"智",11*/

{"电",  0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x3F, 0xF8, 0x21, 0x08, 0x21, 0x08, 0x21, 0x08, 0x3F, 0xF8, 
        0x21, 0x08, 0x21, 0x08, 0x21, 0x08, 0x3F, 0xF8, 0x21, 0x0A, 0x01, 0x02, 0x01, 0x02, 0x00, 0xFE }, /*"电",12*/

{"子",  0x00, 0x00, 0x7F, 0xF8, 0x00, 0x10, 0x00, 0x20, 0x00, 0x40, 0x01, 0x80, 0x01, 0x00, 0xFF, 0xFE, 
        0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00 }, /*"子",13*/

{"模",  0x11, 0x10, 0x11, 0x14, 0x1F, 0xFE, 0x11, 0x10, 0xFD, 0x18, 0x13, 0xFC, 0x32, 0x08, 0x3B, 0xF8,
        0x56, 0x08, 0x53, 0xF8, 0x90, 0x40, 0x1F, 0xFE, 0x10, 0x40, 0x10, 0xB0, 0x11, 0x0E, 0x16, 0x04},

{"式",  0x00, 0x80, 0x00, 0xA0, 0x00, 0x90, 0x00, 0x84, 0xFF, 0xFE, 0x00, 0x80, 0x00, 0x80, 0x3E, 0x80,
        0x08, 0x80, 0x08, 0x40, 0x08, 0x40, 0x09, 0x20, 0x0E, 0x22, 0x70, 0x12, 0x20, 0x0A, 0x00, 0x04},

{"选",  0x00, 0x40, 0x42, 0x40, 0x32, 0x48, 0x13, 0xFC, 0x02, 0x40, 0x04, 0x44, 0xF7, 0xFE, 0x10, 0xA0,
        0x10, 0xA0, 0x10, 0xA0, 0x11, 0x22, 0x11, 0x22, 0x12, 0x1E, 0x2C, 0x00, 0x44, 0x06, 0x03, 0xFC},

{"择",  0x10, 0x00, 0x13, 0xF8, 0x11, 0x10, 0x10, 0xA0, 0xFC, 0x40, 0x10, 0xA0, 0x15, 0x10, 0x1A, 0x4E,
        0x35, 0xF4, 0xD0, 0x40, 0x10, 0x48, 0x17, 0xFC, 0x10, 0x40, 0x10, 0x40, 0x50, 0x40, 0x20, 0x40},

{"单",  0x10, 0x10, 0x08, 0x20, 0x04, 0x48, 0x3F, 0xFC, 0x21, 0x08, 0x21, 0x08, 0x3F, 0xF8, 0x21, 0x08, 
        0x21, 0x08, 0x3F, 0xF8, 0x21, 0x00, 0x01, 0x04, 0xFF, 0xFE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00}, 

{"音",  0x02, 0x00, 0x01, 0x10, 0x3F, 0xF8, 0x08, 0x20, 0x04, 0x20, 0x04, 0x44, 0xFF, 0xFE, 0x00, 0x10, 
        0x1F, 0xF8, 0x10, 0x10, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10}, 

{"数",  0x08, 0x40, 0x49, 0x40, 0x2A, 0x40, 0x08, 0x84, 0xFF, 0xFE, 0x19, 0x08, 0x2C, 0x88, 0x4A, 0x88, 
        0x10, 0x88, 0xFE, 0x90, 0x22, 0x50, 0x22, 0x20, 0x14, 0x50, 0x18, 0x88, 0x25, 0x0E, 0x42, 0x04}, 

{"字",  0x02, 0x00, 0x01, 0x00, 0x3F, 0xFC, 0x20, 0x04, 0x40, 0x08, 0x1F, 0xE0, 0x00, 0x40, 0x00, 0x80, 
        0x01, 0x04, 0xFF, 0xFE, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00}, 

{"斜",  0x08, 0x08, 0x08, 0x08, 0x14, 0x08, 0x22, 0x48, 0x41, 0x28, 0xBE, 0x08, 0x08, 0x48, 0x08, 0x28, 
        0x7F, 0x0E, 0x08, 0xF8, 0x2A, 0x08, 0x29, 0x08, 0x49, 0x08, 0x08, 0x08, 0x28, 0x08, 0x10, 0x08}, 

{"坡",  0x10, 0x20, 0x10, 0x20, 0x10, 0x20, 0x13, 0xFE, 0xFE, 0x22, 0x12, 0x24, 0x12, 0x20, 0x13, 0xFC, 
        0x12, 0x84, 0x12, 0x88, 0x1E, 0x48, 0xE2, 0x50, 0x42, 0x20, 0x04, 0x50, 0x08, 0x8E, 0x13, 0x04}, 

{"调",  0x40, 0x04, 0x27, 0xFE, 0x24, 0x44, 0x04, 0x44, 0x05, 0xF4, 0xE4, 0x44, 0x24, 0x54, 0x27, 0xFC, 
        0x24, 0x04, 0x25, 0xF4, 0x25, 0x14, 0x2D, 0x14, 0x35, 0xF4, 0x25, 0x04, 0x08, 0x14, 0x10, 0x08}, 

{"制",  0x04, 0x04, 0x24, 0x04, 0x25, 0x04, 0x3F, 0xA4, 0x24, 0x24, 0x44, 0xA4, 0xFF, 0xE4, 0x04, 0x24, 
        0x3F, 0xA4, 0x24, 0xA4, 0x24, 0xA4, 0x24, 0x84, 0x26, 0x84, 0x25, 0x04, 0x04, 0x14, 0x04, 0x08}, 

{"并",  0x10, 0x10, 0x0C, 0x30, 0x04, 0x40, 0x3F, 0xFC, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x24, 
        0xFF, 0xFE, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x08, 0x20, 0x08, 0x20, 0x10, 0x20, 0x20, 0x20}, 

{"行",  0x08, 0x08, 0x0B, 0xFC, 0x10, 0x00, 0x20, 0x00, 0x40, 0x00, 0x08, 0x04, 0x17, 0xFE, 0x30, 0x10, 
        0x50, 0x10, 0x90, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x50, 0x10, 0x20}, 

{"据",  0x10, 0x04, 0x13, 0xFE, 0x12, 0x04, 0x12, 0x04, 0xFF, 0xFC, 0x12, 0x20, 0x16, 0x24, 0x1B, 0xFE, 
        0x32, 0x20, 0xD2, 0x24, 0x13, 0xFE, 0x15, 0x04, 0x15, 0x04, 0x15, 0x04, 0x59, 0xFC, 0x21, 0x04}, 

{"端",  0x00, 0x20, 0x20, 0x20, 0x11, 0x24, 0x11, 0x24, 0xFD, 0x24, 0x01, 0xFC, 0x48, 0x00, 0x4B, 0xFE, 
        0x48, 0x44, 0x49, 0xFE, 0x49, 0x54, 0x11, 0x54, 0x1D, 0x54, 0xE1, 0x54, 0x41, 0x04, 0x01, 0x0C}, 

{"口",  0x00, 0x00, 0x00, 0x08, 0x3F, 0xFC, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 
        0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x20, 0x08, 0x3F, 0xF8, 0x20, 0x08, 0x00, 0x00, 0x00, 0x00},

{"、",  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x1E, 0x00, 0x0F, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00},

{"向",  0x02, 0x00, 0x04, 0x00, 0x08, 0x04, 0x7F, 0xFE, 0x40, 0x04, 0x40, 0x24, 0x4F, 0xF4, 0x48, 0x24, 
        0x48, 0x24, 0x48, 0x24, 0x48, 0x24, 0x4F, 0xE4, 0x48, 0x24, 0x40, 0x04, 0x40, 0x14, 0x40, 0x08}, 

{"上",  0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x10, 0x01, 0xF8, 0x01, 0x00, 0x01, 0x00, 
        0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0xFF, 0xFE, 0x00, 0x00}, 

{"下",  0x00, 0x04, 0xFF, 0xFE, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x80, 0x02, 0x40, 0x02, 0x30, 
        0x02, 0x10, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00}, 

{"确",  0x00, 0x40, 0x04, 0x40, 0xFE, 0x78, 0x10, 0x90, 0x11, 0x24, 0x23, 0xFE, 0x25, 0x24, 0x7D, 0x24, 
        0xA5, 0xFC, 0x25, 0x24, 0x25, 0x24, 0x25, 0xFC, 0x25, 0x24, 0x3D, 0x24, 0x22, 0x24, 0x04, 0x0C}, 

{"定",  0x02, 0x00, 0x01, 0x00, 0x7F, 0xFE, 0x40, 0x02, 0x80, 0x24, 0x1F, 0xF0, 0x01, 0x00, 0x01, 0x00, 
        0x11, 0x20, 0x11, 0xF0, 0x11, 0x00, 0x11, 0x00, 0x11, 0x00, 0x29, 0x06, 0x47, 0xFC, 0x80, 0x00}, 

{"返",  0x00, 0x08, 0x40, 0x1C, 0x33, 0xE0, 0x12, 0x00, 0x02, 0x00, 0x02, 0xF8, 0xF2, 0x08, 0x12, 0x90, 
        0x12, 0x50, 0x12, 0x20, 0x12, 0x50, 0x14, 0x8C, 0x15, 0x04, 0x28, 0x00, 0x44, 0x06, 0x03, 0xFC}, 

{"回",  0x00, 0x00, 0x00, 0x04, 0x7F, 0xFE, 0x40, 0x04, 0x40, 0x44, 0x47, 0xE4, 0x44, 0x44, 0x44, 0x44, 
        0x44, 0x44, 0x44, 0x44, 0x47, 0xC4, 0x44, 0x44, 0x40, 0x04, 0x7F, 0xFC, 0x40, 0x04, 0x00, 0x00},

{"度",  0x01, 0x00, 0x00, 0x84, 0x3F, 0xFE, 0x22, 0x20, 0x22, 0x28, 0x3F, 0xFC, 0x22, 0x20, 0x23, 0xE0, 
        0x20, 0x00, 0x2F, 0xF0, 0x22, 0x20, 0x21, 0x40, 0x20, 0x80, 0x43, 0x60, 0x8C, 0x1E, 0x30, 0x04}, 

{"相",  0x10, 0x04, 0x11, 0xFE, 0x11, 0x04, 0x11, 0x04, 0xFD, 0x04, 0x11, 0xFC, 0x31, 0x04, 0x39, 0x04, 
        0x55, 0x04, 0x51, 0xFC, 0x91, 0x04, 0x11, 0x04, 0x11, 0x04, 0x11, 0x04, 0x11, 0xFC, 0x11, 0x04}, 

{"位",  0x08, 0x80, 0x08, 0x40, 0x08, 0x40, 0x10, 0x08, 0x17, 0xFC, 0x30, 0x00, 0x52, 0x08, 0x92, 0x08, 
        0x11, 0x10, 0x11, 0x10, 0x10, 0x90, 0x10, 0xA0, 0x10, 0x24, 0x1F, 0xFE, 0x10, 0x00, 0x10, 0x00},	

{"直",  0x01, 0x00, 0x01, 0x08, 0x7F, 0xFC, 0x01, 0x00, 0x01, 0x10, 0x1F, 0xF8, 0x10, 0x10, 0x1F, 0xF0, 
        0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x1F, 0xF0, 0x10, 0x10, 0x10, 0x14, 0xFF, 0xFE, 0x00, 0x00}, 

{"接",  0x10, 0x80, 0x10, 0x48, 0x17, 0xFC, 0x10, 0x00, 0xFD, 0x10, 0x10, 0xA0, 0x17, 0xFC, 0x18, 0x80, 
        0x30, 0x84, 0xDF, 0xFE, 0x10, 0x90, 0x11, 0x10, 0x10, 0xA0, 0x10, 0x60, 0x50, 0x98, 0x23, 0x08}, 

{"转",  0x10, 0x40, 0x10, 0x40, 0x10, 0x48, 0xFD, 0xFC, 0x20, 0x40, 0x28, 0x44, 0x4B, 0xFE, 0x7C, 0x80, 
        0x08, 0x80, 0x09, 0xFC, 0x1C, 0x08, 0xE8, 0x10, 0x48, 0x90, 0x08, 0x60, 0x08, 0x20, 0x08, 0x10}, 

{"换",  0x10, 0x80, 0x10, 0x80, 0x11, 0xF0, 0x12, 0x20, 0xFC, 0x48, 0x13, 0xFC, 0x12, 0x48, 0x1E, 0x48, 
				0x32, 0x48, 0xD2, 0x48, 0x1F, 0xFE, 0x10, 0x40, 0x10, 0xA0, 0x11, 0x10, 0x52, 0x0E, 0x2C, 0x04}, 

{"双",  0x00, 0x00, 0x00, 0x04, 0xFD, 0xFE, 0x04, 0x84, 0x44, 0x84, 0x44, 0x84, 0x28, 0x88, 0x28, 0x48, 
				0x10, 0x48, 0x10, 0x50, 0x28, 0x20, 0x28, 0x30, 0x44, 0x50, 0x44, 0x88, 0x81, 0x0E, 0x06, 0x04}, 

{"连",  0x00, 0x80, 0x40, 0x88, 0x2F, 0xFC, 0x21, 0x00, 0x01, 0x40, 0x02, 0x50, 0xE7, 0xF8, 0x20, 0x40, 
				0x20, 0x40, 0x20, 0x48, 0x2F, 0xFC, 0x20, 0x40, 0x20, 0x40, 0x50, 0x46, 0x8F, 0xFC, 0x00, 0x00}, 

{"续",  0x10, 0x40, 0x10, 0x50, 0x23, 0xF8, 0x20, 0x40, 0x48, 0x40, 0xFF, 0xFC, 0x11, 0x24, 0x20, 0xA8, 
				0x42, 0x20, 0xF9, 0x24, 0x07, 0xFE, 0x00, 0x40, 0x1C, 0x60, 0xE0, 0x90, 0x41, 0x0C, 0x02, 0x04}, 

{"循",  0x10, 0x1C, 0x17, 0xE0, 0x24, 0x20, 0x24, 0x24, 0x47, 0xFE, 0x94, 0x20, 0x25, 0xFC, 0x65, 0x04, 
				0xA5, 0x04, 0x25, 0xFC, 0x25, 0x04, 0x25, 0xFC, 0x29, 0x04, 0x29, 0x04, 0x31, 0xFC, 0x21, 0x04}, 

{"环",  0x10, 0x00, 0xF8, 0x04, 0x27, 0xFE, 0x20, 0x20, 0x20, 0x20, 0x20, 0x40, 0xF8, 0x40, 0x20, 0xD0, 
				0x21, 0x4C, 0x22, 0x46, 0x24, 0x42, 0x20, 0x40, 0x38, 0x40, 0xE0, 0x40, 0x40, 0x40, 0x00, 0x40},

{"升",  0x01, 0x20, 0x07, 0xA0, 0x7C, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x24, 0xFF, 0xFE, 
        0x04, 0x20, 0x04, 0x20, 0x04, 0x20, 0x08, 0x20, 0x08, 0x20, 0x10, 0x20, 0x20, 0x20, 0x00, 0x20}, 

{"降",  0x00, 0x80, 0x7C, 0x80, 0x45, 0xF8, 0x4A, 0x10, 0x48, 0xA0, 0x50, 0x40, 0x49, 0xB0, 0x4A, 0x4E, 
        0x44, 0x44, 0x45, 0xF0, 0x45, 0x40, 0x69, 0x48, 0x53, 0xFC, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40}, 

{"步",  0x01, 0x00, 0x09, 0x00, 0x09, 0x10, 0x09, 0xF8, 0x09, 0x00, 0x09, 0x04, 0xFF, 0xFE, 0x01, 0x00, 
        0x09, 0x10, 0x0D, 0x18, 0x11, 0x20, 0x21, 0x20, 0x00, 0xC0, 0x03, 0x00, 0x0C, 0x00, 0x70, 0x00}, 

{"进",  0x02, 0x20, 0x42, 0x20, 0x22, 0x28, 0x2F, 0xFC, 0x02, 0x20, 0x02, 0x20, 0xE2, 0x20, 0x22, 0x28, 
        0x2F, 0xFC, 0x22, 0x20, 0x22, 0x20, 0x22, 0x20, 0x24, 0x20, 0x50, 0x26, 0x8F, 0xFC, 0x00, 0x00}, 

{"大",  0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0xFF, 0xFE, 0x01, 0x00, 0x02, 0x80, 
        0x02, 0x80, 0x02, 0x40, 0x04, 0x40, 0x04, 0x20, 0x08, 0x10, 0x10, 0x0E, 0x60, 0x04, 0x00, 0x00}, 

{"小",  0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x05, 0x40, 0x05, 0x20, 0x09, 0x10, 
        0x09, 0x08, 0x11, 0x04, 0x21, 0x04, 0x41, 0x00, 0x01, 0x00, 0x01, 0x00, 0x05, 0x00, 0x02, 0x00}, 

{"时",  0x00, 0x08, 0x04, 0x08, 0x7E, 0x08, 0x44, 0x08, 0x47, 0xFE, 0x44, 0x08, 0x44, 0x08, 0x7C, 0x88, 
        0x44, 0x48, 0x44, 0x48, 0x44, 0x08, 0x44, 0x08, 0x7C, 0x08, 0x44, 0x48, 0x00, 0x28, 0x00, 0x10}, 

{"间",  0x20, 0x04, 0x1B, 0xFE, 0x08, 0x04, 0x40, 0x24, 0x4F, 0xF4, 0x48, 0x24, 0x48, 0x24, 0x48, 0x24, 
        0x4F, 0xE4, 0x48, 0x24, 0x48, 0x24, 0x48, 0x24, 0x4F, 0xE4, 0x48, 0x24, 0x40, 0x14, 0x40, 0x08},

{"起",  0x08, 0x00, 0x08, 0x08, 0x08, 0xFC, 0x7E, 0x08, 0x08, 0x08, 0x08, 0x08, 0xFE, 0xF8, 0x08, 0x88, 
        0x28, 0x80, 0x2E, 0x84, 0x28, 0x84, 0x28, 0x7C, 0x28, 0x00, 0x58, 0x06, 0x8F, 0xFC, 0x00, 0x00}, 

{"始",  0x10, 0x40, 0x10, 0x40, 0x10, 0x40, 0x10, 0x80, 0xFC, 0x88, 0x25, 0x04, 0x27, 0xFE, 0x24, 0x02, 
        0x24, 0x04, 0x49, 0xFE, 0x29, 0x04, 0x11, 0x04, 0x29, 0x04, 0x45, 0x04, 0x85, 0xFC, 0x01, 0x04}, 

{"终",  0x10, 0x80, 0x10, 0x80, 0x20, 0xFC, 0x21, 0x08, 0x45, 0x90, 0xFA, 0x50, 0x10, 0x20, 0x20, 0x50, 
        0x40, 0x88, 0xFD, 0x0E, 0x02, 0x64, 0x00, 0x10, 0x1C, 0x08, 0xE0, 0xC0, 0x40, 0x30, 0x00, 0x08}, 

{"止",  0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x11, 0x00, 0x11, 0x00, 0x11, 0x10, 0x11, 0xF8, 0x11, 0x00, 
        0x11, 0x00, 0x11, 0x00, 0x11, 0x00, 0x11, 0x00, 0x11, 0x00, 0x11, 0x04, 0xFF, 0xFE, 0x00, 0x00},

{"权",  0x10, 0x00, 0x10, 0x00, 0x13, 0xF8, 0x10, 0x08, 0xFE, 0x08, 0x12, 0x08, 0x31, 0x08, 0x39, 0x10,
        0x55, 0x10, 0x50, 0xA0, 0x90, 0x40, 0x10, 0xA0, 0x11, 0x10, 0x12, 0x0E, 0x14, 0x04, 0x10, 0x00},
};




sFONT_CN Font16_CN = {
  codeGB_16,
  16, /* Width */
  16, /* Height */
  sizeof(codeGB_16) / sizeof(codeGB_16[0])
};



#endif 

