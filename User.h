#pragma once

float hypo3(float a, float b, float c )
{
	return sqrt(a*a + b*b + c*c);
}

float filtr(int16_t curr_val, float prev_val, uint8_t mod) {
	return ((curr_val + prev_val * mod)/(1 + mod));
}