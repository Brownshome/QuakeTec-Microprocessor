#include <InternalADC/QT_adc_internal.h>
#include "driverlib.h"

#include "SpiLib/QT_SPI_Protocol.h"
#include "SpiLib/QT_SPI_SpiLib.h"
#include "QT_LPMain.h"

#include "ExternalADC/QT_adc_external.h"
#include "Timer/QT_timer.h"

#define EVENT_QUEUE_LENGTH 256
#define EVENT_QUEUE_HEADER_LENGTH 2

// Event buffer code. The buffer is on
byte eventQueue[EVENT_QUEUE_LENGTH + EVENT_QUEUE_HEADER_LENGTH];
int eventQueueStart = EVENT_QUEUE_LENGTH;
volatile bool finishedSendingEvents = true;

// Sweep values
uint16_t dacValue;

uint8_t digipotControl;
uint8_t digipotData;

// Command systems
volatile bool exitCommand = false;

/** The value of this variable is undefined if commandRunning = false */
volatile PL_Command_t currentCommand;
volatile bool commandRunning = false;

/**
 * Setup the libraries and IO pins.
 */
void initialise() {
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    CS_initFLLSettle(
            16000,
            487
    );

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1);

    //QT_IADC_initialise();
    QT_SPI_initialise();
    //QT_TIMER_initialise();
    QT_EADC_initialise();

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

void finishTransmission(bool unused) {
    QT_SPI_listenToMaster();
}

void flagEventsFinishedSending(bool unused) {
    finishedSendingEvents = true;

    finishTransmission(unused);
}

void sendEvents() {
    eventQueue[eventQueueStart - 1] = EVENT_QUEUE_LENGTH - eventQueueStart;
    eventQueue[eventQueueStart - 2] = PL_START_BYTE;

    finishedSendingEvents = false;

    QT_SPI_stopListeningToMaster(); // Stop reading until the data is sent

    QT_SPI_transmit(
            &eventQueue[eventQueueStart - EVENT_QUEUE_HEADER_LENGTH],
            EVENT_QUEUE_LENGTH - eventQueueStart + EVENT_QUEUE_HEADER_LENGTH,
            &OBC,
            flagEventsFinishedSending);

    eventQueueStart = EVENT_QUEUE_LENGTH;
}

byte queryData [4] = { PL_START_BYTE, 2, 0, 0 };

void sendMax() {

}

void sendMin() {

}

void sendDigipot() {

}

void sendTemperature() {
    QT_SPI_stopListeningToMaster();

    queryData[2] = 0xf0;
    queryData[3] = 0x0f;

    QT_SPI_transmit(queryData, 4, &OBC, finishTransmission);
}

void sendSamplingData() {

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
    if(commandRunning && (command == PL_COMMAND_POWER_OFF/* || command == PL_COMMAND_STOP_TASK */)) {
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
    QT_SPI_listenToMaster();
}

volatile bool retrigger = true;

void voltage(float number) {
    retrigger = true;
}

/*
 * This file handles the communication with the OBC, this is the main event loop that handles
 */
void main(void) {
    initialise();

    //startListening();

    while(true) {
        QT_EADC_measureFloatVoltage(voltage);
        __delay_cycles(1000000);
    }

    while(true) {
        // Wait until a command has been queued
        while(!commandRunning);

        // These command functions are blocking.
        switch(currentCommand) {
        case PL_COMMAND_CALIBRATION_START:
            break;
        case PL_COMMAND_CALIBRATION_STOP:
            break;
            //case PL_COMMAND_DEPLOY:
            //    break;
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
