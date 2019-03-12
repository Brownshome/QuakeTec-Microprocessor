#ifndef QT_SW_SWEEP_H_
#define QT_SW_SWEEP_H_

/*
 * Includes
 */
#include "Common/QT_COM_common.h"
#include "ExternalADC/QT_adc_external.h"
#include "DAC/QT_DAC.h"
#include "Digipot/QT_digipot.h"
#include "FRAM/QT_FRAM.h"

/*
 * Type definitions
 */

typedef struct {
    float digiPotGain;
    uint16_t minDacVoltage;
    uint16_t maxDacVoltage;
    int dacOffset;
} sweep_settings_t;

typedef struct {
    /** A pointer to the array of sweep voltages. This will be in FRAM, and should therefore not be written to. */
    uint16_t *sweepVoltages;

    /** The number of floats in the array. */
    uint16_t bufferLength;
} sweep_data_t;

/*
 * Public functions
 */

/**
 * Initialises DigiPot and DAC before main sweep, returning the settings object that represents.
 */
sweep_settings_t QT_SW_conductPreSweep();

/**
 * Conducts a sweep. When sweep data is available, conductSweep() tells the OBC by calling QT_LP_signalSweepDataReady().
 */
sweep_data_t QT_SW_conductSweep(sweep_settings_t settings);

#endif /* QT_SW_SWEEP_H_ */
