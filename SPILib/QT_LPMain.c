#include <limits.h>

#include "driverlib.h"

#include "SpiLib/QT_SPI_Protocol.h"
#include "SpiLib/QT_SPI_SpiLib.h"
#include "InternalADC/QT_adc.h"
#include "BurnWire/QT_BW_BurnWire.h"
#include "QT_LPMain.h"

#define EVENT_QUEUE_LENGTH 250
#define EVENT_QUEUE_HEADER_LENGTH 2

// Event buffer code. The buffer is on
byte eventQueue [EVENT_QUEUE_LENGTH + EVENT_QUEUE_HEADER_LENGTH];
int eventQueueStart = EVENT_QUEUE_LENGTH;
volatile bool finishedSendingEvents = true;

// Send buffer
byte dataBuffer [PL_QUERY_MAX_LENGTH];

// Sweep values
uint16_t dacValue;

uint8_t digipotControl;
uint8_t digipotData;

// Sweep data
uint16_t *voltageArray = NULL;
uint16_t bufferLength;

// Command systems
volatile bool exitCommand = false;

// Temperature Convertion
const float maximumTemperature = -100.0;
const float minimumTemperature = 100.0;

/** The value of this variable is undefined if commandRunning = false */
volatile PL_Command_t currentCommand;
volatile bool commandRunning = false;

/**
 * Setup the libraries and IO pins.
 */
void initialise() {
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN1);

    QT_ADC_initialise();
    QT_SPI_initialise();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    // Enable interrupts
    __enable_interrupt();
}

/**
 * Adds an event to the list.
 *
 * TODO Shout at the OBC if we overflow
 **/
void queueEvent(PL_Event_t event) {
    if(eventQueueStart <= 0 || !finishedSendingEvents) {
        // Overflow. This should not occur givent he OBC does not send more than 256 commands.
        return;
    }

    eventQueue[--eventQueueStart] = (byte) event;
}

void flagEventsFinishedSending(bool unused) {
    finishedSendingEvents = true;
}

void sendEvents() {
    eventQueue[eventQueueStart - 1] = EVENT_QUEUE_LENGTH - eventQueueStart;
    eventQueue[eventQueueStart - 2] = PL_START_BYTE;

    finishedSendingEvents = false;

    QT_SPI_transmit(
            &eventQueue[eventQueueStart - EVENT_QUEUE_HEADER_LENGTH],
            EVENT_QUEUE_LENGTH - eventQueueStart + EVENT_QUEUE_HEADER_LENGTH,
            &OBC,
            flagEventsFinishedSending);

    eventQueueStart = EVENT_QUEUE_LENGTH;
}

void sendMax() {

}

void sendMin() {

}

void sendDigipot() {

}

void sendTemperature() {
    float degreesC = QT_ADC_readTemperature();

    float range = maximumTemperature - minimumTemperature;
    float correctedValue = (degreesC - minimumTemperature) / range;

    if(correctedValue < 0.0) {
        correctedValue = 0.0;
    } else if(correctedValue > 1.0) {
        correctedValue = 1.0;
    }

    uint16_t rawData = correctedValue * UINT16_MAX;

    // Assume that the OBC will not ask for data while we are sending data.
    dataBuffer[0] = PL_START_BYTE;
    dataBuffer[1] = PL_TEMP_SIZE;

    // Match endianess to the processor
    uint16_t *castPtr = (uint16_t *) &dataBuffer[2];
    *castPtr = rawData;

    // Send data
    while(!QT_SPI_transmit(dataBuffer, 4, &ADC, NULL));
}

void sendSamplingData() {
    //sweep_data_t result = QT_SW_retreiveSweepData();
}

void handleQuery(PL_Query_t query, const byte data [2]) {
    switch(query) {
    case PL_QUERY_EVENT:
        sendEvents();
        break;
    case PL_QUERY_PROBE_DAC_MAX:
        sendMax();
        break;
    case PL_QUERY_PROBE_DAC_MIN:
        sendMin();
        break;
    case PL_QUERY_PROBE_DIGIPOT:
        sendDigipot();
        break;
    case PL_QUERY_PROBE_TEMPERATURE:
        sendTemperature();
        break;
    case PL_QUERY_SAMPLING_DATA:
        sendSamplingData();
        break;
    }
}

void handleCommand(PL_Command_t command, const byte data [2]) {
    if(commandRunning && (command == PL_COMMAND_POWER_OFF || command == PL_COMMAND_STOP_TASK)) {
        exitCommand = true;
        queueEvent(PL_EVENT_TASK_INTERRUPTED);

        // Wait for the command to halt to ensure that the new value is read by the event loop
        while(commandRunning);
    }

    currentCommand = command;
    commandRunning = true;
}

void obcIncommingHandler(const byte *data) {
    byte code = data[0];

    if(code < PL_COMMAND_ENUM_COUNT) {
        handleCommand((PL_Command_t) code, data + 1);
    } else {
        handleQuery((PL_Query_t) code, data + 1);
    }
}

/**
 * Start the SPI communication listening lines.
 */
void startListening() {
    QT_SPI_setReceiveHandler(obcIncommingHandler, 2, &OBC);
}

/*
 * This file handles the communication with the OBC, this is the main event loop that handles
 */
void main(void) {
    initialise();

    startListening();

    while(true) {
        // Wait until a command has been queued
        while(!commandRunning) {
            sendTemperature();
        }

        // These command functions are blocking.
        switch(currentCommand) {
        case PL_COMMAND_CALIBRATION_START:
            break;
        case PL_COMMAND_CALIBRATION_STOP:
            break;
        case PL_COMMAND_DEPLOY:
            QT_BW_deploy();
            break;
        case PL_COMMAND_POWER_OFF:
            break;
        case PL_COMMAND_POWER_ON:
            break;
        case PL_COMMAND_SAMPLING_START:
            break;
        case PL_COMMAND_SAMPLING_STOP:
            break;
        default:
            break;
        }

        commandRunning = false;
    }
}
