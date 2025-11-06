#ifndef Q_TABLE_H
#define Q_TABLE_H

#include <stdint.h>

#define NUM_STATES 7
#define NUM_ACTIONS 2

const float Q_TABLE[NUM_STATES][NUM_ACTIONS] = {
    {50.652070f, 158.763369f},
    {53.139640f, 144.473229f},
    {147.852300f, 37.010547f},
    {151.618207f, 114.150829f},
    {103.147624f, 170.540311f},
    {13.384660f, -12.906603f},
    {110.480887f, -5.340473f}
};

const uint8_t OPTIMAL_POLICY[NUM_STATES] = {1, 1, 0, 0, 1, 0, 0};

#endif
