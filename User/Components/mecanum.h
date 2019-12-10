#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float direction;
	
	float left_front;
	float left_rear;
	
	float right_front;
	float right_rear;
} Mecanum_t;

void Mecanum_Init(Mecanum_t* hmecanum);
void Mecanum_Update(Mecanum_t* hmecanum);
