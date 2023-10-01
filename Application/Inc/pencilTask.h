/*
 * pencilTask.h
 *
 *  Created on: Aug 2, 2023
 *      Author: LBQ
 */

#pragma once

extern struct Square_t PencilFrame;

extern void readAllPointFlash(void);
extern void writePointFlash(uint16_t point);
extern void writeAllPointFlash(void);

extern bool_t PencilScanDoneFlag;
