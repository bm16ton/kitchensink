#ifndef SEESAWNEO_H
#define SEESAWNEO_H

void clearnwrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue);

void neowrite(uint8_t start, uint8_t start2, uint8_t green, uint8_t red, uint8_t blue);

void seesawneoint(uint8_t numpix);

void clearseesaw(uint8_t numofneo);

void neoeveryother(uint8_t green, uint8_t red, uint8_t blue, uint8_t green2, uint8_t red2, uint8_t blue2);

void neodown(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue);

void neoup(uint8_t pins, uint8_t green, uint8_t red, uint8_t blue);

#endif
