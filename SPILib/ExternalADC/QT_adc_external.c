/*
 * QT_adc_external.c
 *
 *  Created on: 24/02/2019
 *      Author: james
 */

#define ADC_LSB 4096
// TODO find this value. This is the analog value that the DAC is driven to.
#define ADC_VA 5.0

#define ADC_SHIFT 1

#include "QT_adc_external.h"
#include "SpiLib/QT_SPI_SpiLib.h"

static adc_read_func_t currentCallback = NULL;

static uint32_t sendData;

static float convertToVoltage(uint16_t raw) {
    // Steps occur in the middle o
    return ADC_VA * raw / ADC_LSB;
}

static void adcHandler(const byte *data) {
    if(currentCallback != NULL) {
        uint16_t rawADCValue = *((uint16_t *) data);

        float result = convertToVoltage(rawADCValue);
        (*currentCallback)(result);
        currentCallback = NULL;
    }
}

/**
 * This must be called after the SPI has initialised
 */
void QT_EADC_initialise() {
    QT_SPI_setReceiveHandler(adcHandler, 2, &ADC);
}

/**
 * Reads the ADC0 voltage.
 * This function returns false if the device was busy. If this is the case
 * no action is taken.
 **/
bool QT_EADC_measureSweepCurrent(adc_read_func_t callback) {
    sendData = 0 << ADC_SHIFT;
    currentCallback = callback;
    return QT_SPI_transmit((byte * ) &sendData, 2, &ADC, NULL);
}

/**
 * Reads the ADC1 voltage.
 * This function returns false if the device was busy. If this is the case
 * no action is taken.
 **/
bool QT_EADC_measureFloatVoltage(adc_read_func_t callback) {
    sendData = 0xffff;
    currentCallback = callback;
    return QT_SPI_transmit((byte *) &sendData, 2, &ADC, NULL);
}

/**
 * Reads the ADC2 voltage.
 * This function returns false if the device was busy. If this is the case
 * no action is taken.
 **/
bool QT_EADC_measureSweepVoltage(adc_read_func_t callback) {
    sendData = 2 << ADC_SHIFT;
    currentCallback = callback;
    return QT_SPI_transmit((byte *) &sendData, 2, &ADC, NULL);
}
