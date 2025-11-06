/* state_classifier.c - Implementasi klasifikasi dan nama state/action */

#include "state_classifier.h"

int8_t classify_state(uint8_t density, uint8_t speed)
{
    if (density > 8 || speed > 50) {
        return -1; // Invalid input
    }

    uint8_t density_cat;
    if (density <= 3) {
        density_cat = 0;      // Rendah
    } else if (density <= 5) {
        density_cat = 1;      // Sedang
    } else {
        density_cat = 2;      // Tinggi
    }

    uint8_t speed_cat;
    if (speed < 30) {
        speed_cat = 0;        // Lambat
    } else if (speed < 40) {
        speed_cat = 1;        // Sedang
    } else {
        speed_cat = 2;        // Cepat
    }

    if (density_cat == 0) {
        if (speed_cat == 1) return 0; // Sepi Lancar
        if (speed_cat == 2) return 1; // Kebut-kebutan
        return -1;
    }
    else if (density_cat == 1) {
        if (speed_cat == 0) return 2; // Padat
        if (speed_cat == 1) return 3; // Ramai Lancar
        if (speed_cat == 2) return 4; // Ramai Cepat
    }
    else if (density_cat == 2) {
        if (speed_cat == 0) return 5; // Macet
        if (speed_cat == 1) return 6; // Padat Merayap
        return -1;
    }

    return -1;
}

const char* get_state_name(int8_t state)
{
    switch(state) {
        case 0: return "Sepi Lancar";
        case 1: return "Kebut-kebutan";
        case 2: return "Padat";
        case 3: return "Ramai Lancar (IDEAL)";
        case 4: return "Ramai Cepat";
        case 5: return "Macet";
        case 6: return "Padat Merayap";
        default: return "Invalid State";
    }
}

const char* get_action_name(uint8_t action)
{
    return (action == 0) ? "TURUNKAN BUMP" : "NAIKKAN BUMP";
}
