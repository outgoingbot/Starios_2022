/*
 * bitmap.h
 *
 *  Created on: 17-Jun-2019
 *      Author: poe
 */

#ifndef BITMAP_H_
#define BITMAP_H_
const unsigned char Boot [] = {
0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00,
0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00,
0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00,
0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x00,
0xA0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x8E, 0x00,
0xA0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x01, 0x80,
0xA0, 0x50, 0x3F, 0x1F, 0xFF, 0x81, 0xF0, 0x0F, 0xF8, 0x0E, 0x00, 0xFC, 0x00, 0xFF, 0x8E, 0x10,
0x40, 0x50, 0x7F, 0x9F, 0xFF, 0x81, 0xF0, 0x0F, 0xFE, 0x0E, 0x07, 0xFF, 0x01, 0xFE, 0x50, 0x10,
0x40, 0x50, 0xFF, 0x9F, 0xFF, 0x83, 0xF0, 0x0F, 0xFF, 0x0E, 0x0F, 0xFF, 0x83, 0xFE, 0x50, 0x28,
0x03, 0x8F, 0xE0, 0x80, 0xF0, 0x03, 0xF8, 0x0F, 0x0F, 0x0E, 0x0F, 0x07, 0xC7, 0x82, 0x50, 0x28,
0x0C, 0x01, 0xE0, 0x00, 0xF0, 0x03, 0xB8, 0x0F, 0x07, 0x0E, 0x1E, 0x03, 0xC7, 0x80, 0x20, 0x28,
0x03, 0x8F, 0xE0, 0x00, 0xF0, 0x07, 0x38, 0x0F, 0x07, 0x0E, 0x1C, 0x01, 0xC7, 0x80, 0x21, 0xC7,
0x00, 0x51, 0xF0, 0x00, 0xF0, 0x07, 0x3C, 0x0F, 0x07, 0x0E, 0x3C, 0x01, 0xE7, 0xC0, 0x06, 0x00,
0x00, 0x50, 0xFC, 0x00, 0xF0, 0x07, 0x1C, 0x0F, 0x0F, 0x0E, 0x3C, 0x01, 0xE3, 0xF0, 0x01, 0xC7,
0x08, 0x50, 0x7F, 0x00, 0xF0, 0x0E, 0x1E, 0x0F, 0xFE, 0x0E, 0x3C, 0x01, 0xE1, 0xFC, 0x00, 0x28,
0x08, 0x20, 0x3F, 0x80, 0xF0, 0x0E, 0x1E, 0x0F, 0xFC, 0x0E, 0x3C, 0x01, 0xE0, 0xFE, 0x04, 0x28,
0x14, 0x20, 0x0F, 0xC0, 0xF0, 0x0E, 0x0E, 0x0F, 0xFC, 0x0E, 0x3C, 0x01, 0xE0, 0x3F, 0x04, 0x28,
0x14, 0x00, 0x03, 0xC0, 0xF0, 0x1F, 0xFF, 0x0F, 0x3E, 0x0E, 0x3C, 0x01, 0xE0, 0x0F, 0x0A, 0x10,
0x14, 0x00, 0x01, 0xC0, 0xF0, 0x1F, 0xFF, 0x0F, 0x1E, 0x0E, 0x1C, 0x01, 0xC0, 0x07, 0x0A, 0x10,
0xE3, 0x80, 0x01, 0xC0, 0xF0, 0x1F, 0xFF, 0x0F, 0x0F, 0x0E, 0x1E, 0x03, 0xC0, 0x07, 0x0A, 0x00,
0x00, 0x61, 0x83, 0xC0, 0xF0, 0x3C, 0x07, 0x8F, 0x0F, 0x0E, 0x1F, 0x07, 0x86, 0x0F, 0x71, 0xC0,
0xE3, 0x85, 0xFF, 0x80, 0xF0, 0x38, 0x07, 0x8F, 0x07, 0x0E, 0x0F, 0xFF, 0x87, 0xFF, 0x80, 0x30,
0x14, 0x05, 0xFF, 0x00, 0xF0, 0x38, 0x03, 0x8F, 0x07, 0x8E, 0x07, 0xFF, 0x07, 0xFC, 0x71, 0xC0,
0x14, 0x0A, 0xFE, 0x00, 0xF0, 0x78, 0x03, 0x8E, 0x03, 0x8E, 0x01, 0xF8, 0x03, 0xF8, 0x0A, 0x00,
0x14, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x04,
0x08, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x04,
0x08, 0x71, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0A,
0x01, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0A,
0x00, 0x71, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A,
0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71,
0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80,
0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71
};


const unsigned char face [] = { //small 16x16 bitmap
0x00, 0x00, 0x03, 0xE0, 0x04, 0x10, 0x08, 0x08, 0x10, 0x04, 0x27, 0xF2, 0x27, 0x72, 0x27, 0x72,
0x20, 0x02, 0x20, 0x02, 0x10, 0x04, 0x08, 0x08, 0x04, 0x10, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00
};


#endif /* BITMAP_H_ */
