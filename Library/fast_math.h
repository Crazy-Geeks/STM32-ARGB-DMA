// stolen from https://github.com/FastLED/FastLED

#pragma once

#include "ch.h"

#define FIXFRAC8(N,D) (((N)*256)/(D))

/// add one byte to another, saturating at 0xFF
/// @param i - first byte to add
/// @param j - second byte to add
/// @returns the sum of i & j, capped at 0xFF
static inline uint8_t qadd8( uint8_t i, uint8_t j)
{
    unsigned int t = i + j;
    if( t > 255) t = 255;
    return t;
}

/// subtract one byte from another, saturating at 0x00
/// @returns i - j with a floor of 0
static inline uint8_t qsub8( uint8_t i, uint8_t j)
{
    int t = i - j;
    if( t < 0) t = 0;
    return t;
}

///         square root for 16-bit integers
///         About three times faster and five times smaller
///         than Arduino's general sqrt on AVR.
static uint8_t sqrt16(uint16_t x)
{
    if( x <= 1) {
        return x;
    }

    uint8_t low = 1; // lower bound
    uint8_t hi, mid;

    if( x > 7904) {
        hi = 255;
    } else {
        hi = (x >> 5) + 8; // initial estimate for upper bound
    }

    do {
        mid = (low + hi) >> 1;
        if ((uint16_t)(mid * mid) > x) {
            hi = mid - 1;
        } else {
            if( mid == 255) {
                return 255;
            }
            low = mid + 1;
        }
    } while (hi >= low);

    return low - 1;
}