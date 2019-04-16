/*
 * Includes
 */
#include <SpiLib/QT_SPI_SpiLib.h>

/*
 * Defines
 */
#define BUFFER_LENGTH 256
#define SPI_CLK_FREQUENCY 32000
#define DEFAULT_SEND 0

typedef void(*spi_transmit_func)(uint16_t, uint8_t);
typedef byte(*spi_receive_func)(uint16_t);

/*
 * Private variables
 */

/**
 * SPI related information about devices to communicate with.
 */
device_t OBC = { false, EUSCI_A1_BASE, NULL, 0, GPIO_PORT_P4, GPIO_PIN0 };

device_t ADC = { true, EUSCI_B1_BASE, NULL, 0, GPIO_PORT_P2, GPIO_PIN5 };
device_t DAC = { true, EUSCI_B1_BASE, NULL, 0, GPIO_PORT_P2, GPIO_PIN4 };
device_t DIGIPOT = { true, EUSCI_B1_BASE, NULL, 0, GPIO_PORT_P2, GPIO_PIN6 };

/**
 * Buffers to hold data before data is ready to send to receive handler
 */
static byte volatile masterReceiveBuffer [BUFFER_LENGTH];
static byte volatile slaveReceiveBuffer [BUFFER_LENGTH];

/**
 * Pointers to next available byte in the buffer. If buffer is full, pointer will point to byte 256.
 */
static byte volatile *volatile masterReceiveBufferPtr;
static byte volatile *volatile slaveReceiveBufferPtr;

/**
 * Pointer to slave device MPS430 communicating with.
 */
static device_t *currentSlavePtr;

/**
 * Whether we are currently listening to the slave.
 */
static bool volatile isListeningToSlave;

static bool volatile isListeningToMaster = true;
static bool volatile requestedMasterListenState = true;

/**
 * This variable is true if we have sent a byte to the transmit function, and we are still waiting for the interrupt to be called.
 */
static bool volatile isSlaveTransmitInterruptPending = false;

/**
 * Pointer to the next byte to be sent, this will be NULL if there is no data to send
 */
static byte const *volatile masterTransmitPtr;
static byte const *volatile slaveTransmitPtr;

/**
 * True if the CS should be shutoff when the data has finished sending.
 */
static volatile bool slaveCSShutoff = false;

/**
 * A pointer to a function that is to be called when the current transmission is completed
 */
static volatile transmit_handler_func slaveTransmitHandler = NULL;
static volatile transmit_handler_func masterTransmitHandler = NULL;

/**
 * A flag to make sure that interrupts do not get disabled inside of interrupts
 */
static volatile bool isInInterrupt = false;

/**
 * Pointers one past the last byte to be sent
 */
static byte const *masterTransmitStopPtr, *slaveTransmitStopPtr;

/*
 * Private functions
 */

/**
 * Sets up the MSP430 as a master device for communication with ADC, DAC and DigiPot.
 */
static void initialiseMaster() {
    const uint8_t port = GPIO_PORT_P4;

    const uint8_t mosi = GPIO_PIN6;
    const uint8_t miso = GPIO_PIN7;
    const uint8_t clk = GPIO_PIN5;

    // Set all of the device chip selects at output
    GPIO_setAsOutputPin(DIGIPOT.csPort, DIGIPOT.csPin);
    GPIO_setAsOutputPin(DAC.csPort, DAC.csPin);
    GPIO_setAsOutputPin(ADC.csPort, ADC.csPin);

    // Deactivate all chip selects
    GPIO_setOutputHighOnPin(DIGIPOT.csPort, DIGIPOT.csPin);
    GPIO_setOutputHighOnPin(DAC.csPort, DAC.csPin);
    GPIO_setOutputHighOnPin(ADC.csPort, ADC.csPin);

    // Select Port 1 - Set Pin 0, Pin 1 and Pin 2 to input Secondary Module Function
    GPIO_setAsPeripheralModuleFunctionInputPin(
            port,
            miso,
            GPIO_PRIMARY_MODULE_FUNCTION
    );

    GPIO_setAsPeripheralModuleFunctionOutputPin(
            port,
            mosi | clk,
            GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Initialize Master
    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = SPI_CLK_FREQUENCY;
    param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    param.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(EUSCI_B1_BASE, &param);

    // Enable SPI module
    EUSCI_B_SPI_enable(EUSCI_B1_BASE);
}

/**
 * Sets up MSP430 as slave device for communication with OBC.
 */
static void initialiseSlave() {
    const uint8_t port = GPIO_PORT_P4;

    const uint8_t mosi = GPIO_PIN3;
    const uint8_t miso = GPIO_PIN2;
    const uint8_t clk = GPIO_PIN1;

    // Set OBC chip select as input
    GPIO_setAsInputPin(OBC.csPort, OBC.csPin);

    // Set MOSI and CLK lines as input
    GPIO_setAsPeripheralModuleFunctionInputPin(
            port,
            mosi + clk,
            GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Set MISO line as output
    GPIO_setAsPeripheralModuleFunctionOutputPin(
            port,
            miso,
            GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Initialize slave to MSB first, inactive high clock polarity and 4 wire SPI
    EUSCI_A_SPI_initSlaveParam param = {0};
    param.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
    param.spiMode = EUSCI_A_SPI_4PIN_UCxSTE_ACTIVE_LOW;
    EUSCI_A_SPI_initSlave(EUSCI_A1_BASE, &param);

    // Select 4 pin functionality
    EUSCI_A_SPI_select4PinFunctionality(EUSCI_A1_BASE, EUSCI_A_SPI_ENABLE_SIGNAL_FOR_4WIRE_SLAVE);

    // Enable SPI Module
    EUSCI_A_SPI_enable(EUSCI_A1_BASE);
}

static void enableAllInterrupts() {
    // Clear receive interrupt flag for communication with slaves
    EUSCI_B_SPI_clearInterrupt(EUSCI_B1_BASE,
                               EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);

    // Enable receive interrupt
    EUSCI_B_SPI_enableInterrupt(EUSCI_B1_BASE,
                                EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);

    // Clear receive interrupt flag for communication with OBC
    EUSCI_A_SPI_clearInterrupt(EUSCI_A1_BASE,
                               EUSCI_A_SPI_RECEIVE_INTERRUPT | EUSCI_A_SPI_TRANSMIT_INTERRUPT);

    // Enable receive interrupt
    EUSCI_A_SPI_enableInterrupt(EUSCI_A1_BASE,
                                EUSCI_A_SPI_RECEIVE_INTERRUPT | EUSCI_A_SPI_TRANSMIT_INTERRUPT);
}

/**
 * Sets the MSP430 to listen to the specified device. If null is passed, then it will clear all chip selects
 *
 * Don't change the currentSlavePtr manually
 *
 * THIS CALL WILL BE LOCKED WITH THE INTERRUPTS FOR THE TO SLAVE CHANNEL
 */
static void setChipSelect(device_t* devicePtr) {
    if (currentSlavePtr != NULL) {
        // Clear cs
        GPIO_setOutputHighOnPin(currentSlavePtr->csPort, currentSlavePtr->csPin);
    }

    currentSlavePtr = devicePtr;

    if (currentSlavePtr != NULL) {
        // Set new cs
        GPIO_setOutputLowOnPin(currentSlavePtr->csPort, currentSlavePtr->csPin);
    }
}


/**
 * This method sends a single 0 to the slave. This method also sets the isSendingToSlave flag.
 *
 * This method is locked by the
 */
static void sendNullByteToSlave() {
    EUSCI_B_SPI_transmitData(currentSlavePtr->spiBaseAddress, DEFAULT_SEND);
    isSlaveTransmitInterruptPending = true;
}

static void sendByteToSlave() {
    if (slaveTransmitPtr == NULL) {
        transmit_handler_func savedTransmitHandler = slaveTransmitHandler;
        slaveTransmitHandler = NULL;

        // If we are listening, but not sending, send 0 to keep the system listening.
        if (isListeningToSlave) {
            sendNullByteToSlave();
        } else {
            // TODO slow propogation of CS but fast propogation of other data
            slaveCSShutoff = true;
        }

        // If there is a handler to call, call it
        if (savedTransmitHandler != NULL) {
            (*savedTransmitHandler)(true); // True for successful transmission
        }

        return;
    }

    // Send the byte
    EUSCI_B_SPI_transmitData(currentSlavePtr->spiBaseAddress, *slaveTransmitPtr);

    isSlaveTransmitInterruptPending = true;

    // Move pointer to the next byte to send
    slaveTransmitPtr++;

    // Have we finished sending the packet?
    if (slaveTransmitPtr == slaveTransmitStopPtr) {
        slaveTransmitPtr = NULL;
    }
}

static void sendByteToMaster() {
    if (masterTransmitPtr == NULL) {
        transmit_handler_func savedTransmitHandler = masterTransmitHandler;
        masterTransmitHandler = NULL;

        // If there is a handler to call, call it
        if (savedTransmitHandler != NULL) {
            (*savedTransmitHandler)(true); // True for successful transmission
        }

        return;
    }

    // Send the byte
    EUSCI_A_SPI_transmitData(OBC.spiBaseAddress, *masterTransmitPtr);

    // Move pointer to the next byte to send
    masterTransmitPtr++;

    // Have we finished sending the packet?
    if (masterTransmitPtr == masterTransmitStopPtr) {
        masterTransmitPtr = NULL;
    }
}

/**
 * Receives a byte of data from a device. Runs a receive handler when all data has been received.
 */
static void receiveByte(device_t *devicePtr, spi_receive_func receiveData, byte volatile *volatile *receiveBufferPtr,
                        byte volatile *receiveBufferStartPtr) {
    // Check if receive handler has been set
    if (devicePtr->receiveHandler == NULL) {
        return;
    }

    // Put received byte of data in slave buffer
    **receiveBufferPtr = (*receiveData)(devicePtr->spiBaseAddress);

    // Move buffer pointer to next byte
    (*receiveBufferPtr)++;

    // If the buffer is full
    if (*receiveBufferPtr - receiveBufferStartPtr >= devicePtr->expectedLength) {

        // Run the handler
        (*(devicePtr->receiveHandler))((byte const *) receiveBufferStartPtr);

        // Clear the buffer
        *receiveBufferPtr = receiveBufferStartPtr;
    }
}

static void disableInterrupts(device_t *device) {
    if (isInInterrupt) {
        return;
    }

    if (device == &OBC) {
        EUSCI_A_SPI_disableInterrupt(OBC.spiBaseAddress, EUSCI_A_SPI_TRANSMIT_INTERRUPT | EUSCI_A_SPI_RECEIVE_INTERRUPT);
    } else {
        EUSCI_B_SPI_disableInterrupt(device->spiBaseAddress, EUSCI_B_SPI_TRANSMIT_INTERRUPT | EUSCI_B_SPI_RECEIVE_INTERRUPT);
    }
}

static void enableInterrupts(device_t *device) {
    if (isInInterrupt) {
        return;
    }

    if (device == &OBC) {
        EUSCI_A_SPI_enableInterrupt(OBC.spiBaseAddress, EUSCI_A_SPI_TRANSMIT_INTERRUPT | EUSCI_A_SPI_RECEIVE_INTERRUPT);
    } else {
        EUSCI_B_SPI_enableInterrupt(device->spiBaseAddress, EUSCI_B_SPI_TRANSMIT_INTERRUPT | EUSCI_B_SPI_RECEIVE_INTERRUPT);
    }
}

/*
 * Public functions
 */

/**
 * Sets up the SPI library
 */
void QT_SPI_initialise() {

    // Set receive buffer to point to start of buffer
    masterReceiveBufferPtr = masterReceiveBuffer;
    slaveReceiveBufferPtr = slaveReceiveBuffer;

    // Set slave device we are listening to as null
    currentSlavePtr = NULL;

    // Set up the send buffers
    slaveTransmitPtr = NULL;
    masterTransmitPtr = NULL;

    // Setup listening
    isListeningToSlave = false;

    // Set base addresses

    initialiseMaster();

    initialiseSlave();

    enableAllInterrupts();
}

/**
 * Returns immediately. Returns true if the system was ready to send data. If it returns false the data was not sent.
 *
 * THIS METHOD CAN BE INTERLEIVED WITH ANY INTERRUPT
 */
bool QT_SPI_transmit(byte const *dataPtr, uint16_t length, device_t *devicePtr, transmit_handler_func handler) {

    disableInterrupts(devicePtr);

    // Check if we are currently sending, if we are return false
    if (devicePtr->isSlave) {
        if (slaveTransmitPtr != NULL) {
            enableInterrupts(devicePtr);
            return false;
        }
    } else {
        if (masterTransmitPtr != NULL) {
            enableInterrupts(devicePtr);
            return false;
        }
    }

    // Check if we are connected to the correct slave, else change and clear
    if (devicePtr->isSlave && currentSlavePtr != devicePtr) {
        // Clear the receive buffer
        slaveReceiveBufferPtr = slaveReceiveBuffer;

        // Select the device
        setChipSelect(devicePtr);

        // Clear the listening flag
        isListeningToSlave = false;
    }

    // Set the transmit buffer
    if (devicePtr->isSlave) {
        slaveTransmitPtr = dataPtr;
        slaveTransmitStopPtr = dataPtr + length;
        slaveTransmitHandler = handler;
    } else {
        masterTransmitPtr = dataPtr;
        masterTransmitStopPtr = dataPtr + length;
        masterTransmitHandler = handler;
    }

    if (!devicePtr->isSlave) {
        // Get the data ready for the next clock to send.
        sendByteToMaster();
    } else if (!isSlaveTransmitInterruptPending) {
        // Start interrupts for the slave, but only if it is not already waiting to send, to avoid overwriting data
        sendByteToSlave();
    }

    enableInterrupts(devicePtr);

    return true;
}

/**
 * Registers a function that handles incoming data. This will be called when length bytes have been recevied.
 *
 * THIS METHOD WILL INTERLIEVE WITH ANY INTERRUPTS
 */
void QT_SPI_setReceiveHandler(receive_handler_func handler, byte length, device_t *devicePtr) {

    // Disable receive interrupts (interrupts will be queued for processing afterwards)
    disableInterrupts(devicePtr);

    // Clear the receive buffers
    masterReceiveBufferPtr = masterReceiveBuffer;
    slaveReceiveBufferPtr = slaveReceiveBuffer;

    // Set receive handler and expected length
    devicePtr->receiveHandler = handler;
    devicePtr->expectedLength = length;

    // Re-enable receive interrupts
    enableInterrupts(devicePtr);
}

/**
 * Starts clocking data in from the slave and sets the CS line. This will cause the handler to be called.
 * If there is data in the receive buffer, clear it.
 *
 * If we are sending data to any slave currently, this method causes us to cease transmission, calling any trasmission handler
 * to be called with a failed status code.
 *
 * THIS METHOD WILL INTERLIEVE WITH ANY INTERRUPTS
 */
void QT_SPI_listenToSlave(device_t *devicePtr) {

    disableInterrupts(devicePtr);

    // Clear data in receive buffer
    slaveReceiveBufferPtr = slaveReceiveBuffer;

    transmit_handler_func savedHandlerFunc = slaveTransmitHandler;
    slaveTransmitHandler = NULL;

    if (slaveTransmitPtr != NULL) {
        slaveTransmitPtr = NULL;
    }

    // Set the chip select
    setChipSelect(devicePtr);

    isListeningToSlave = true;

    if (!isSlaveTransmitInterruptPending) {
        sendNullByteToSlave();
    }

    // If we currently are transmiting to the slave we need to halt the transmission and call the handler with a failed transmission result.
    if (savedHandlerFunc != NULL) {
        (*savedHandlerFunc)(false);
    }

    enableInterrupts(devicePtr);
}

/**
 * Stops listening to the current slave.
 *
 * THIS METHOD WILL INTERLIEVE WITH ANY INTERRUPTS
 */
void QT_SPI_stopListeningToSlave() {
    isListeningToSlave = false;
}

/**
 * This function is called to propogate isListening, this is to guard against this state changing while
 * the byte is sending.
 */
static void setMasterListenState(bool isListening) {
    if(isListeningToMaster == isListening) {
        return;
    }

    isListeningToMaster = isListening;

    if(isListeningToMaster) {
        // Reset the buffer
        masterReceiveBufferPtr = masterReceiveBuffer;
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

/**
 * Stops listening to the master.
 *
 * THIS METHOD WILL INTERLIEVE WITH ANY INTERRUPTS
 */
void QT_SPI_stopListeningToMaster() {
    requestedMasterListenState = false;
}

/**
 * Starts listening to master.
 *
 * THIS METHOD WILL INTERLIEVE WITH ANY INTERRUPTS
 */
void QT_SPI_listenToMaster() {
    requestedMasterListenState = true;
}

/**
 * Returns true if all data has been sent.
 *
 * THIS METHOD WILL INTERLIEVE WITH ANY INTERRUPTS
 */
bool QT_SPI_isDataSent() {
    return masterTransmitPtr == NULL && slaveTransmitPtr == NULL;
}

/*
 * Interrupts
 */

/**
 * Interrupt for processing received and transmitted data from the B1 channel (from slave devices)
 */
#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
    switch(UCB1IV)
    {
    // Receive data case
    case USCI_SPI_UCRXIFG:
        // Check if we're pointing to a slave
        if (currentSlavePtr == NULL) {
            break;
        }

        receiveByte(currentSlavePtr, &EUSCI_B_SPI_receiveData, &slaveReceiveBufferPtr, slaveReceiveBuffer);

        if(slaveCSShutoff) {
            setChipSelect(NULL);
            slaveCSShutoff = false;
        }

        break;

        // Transmit data case
    case USCI_SPI_UCTXIFG:
        isSlaveTransmitInterruptPending = false;

        if (currentSlavePtr == NULL) {
            break;
        }

        sendByteToSlave();

        break;

    default:
        break;
    }
}

/**
 * Interrupt for processing transmitted and received data from OBC
 */
#pragma vector=USCI_A1_VECTOR
__interrupt
void USCI_A1_ISR (void)
{
    switch(UCA1IV)
        {
            // Receive data case
            case USCI_SPI_UCRXIFG:
                // Receive data from master
                if(isListeningToMaster) {
                    receiveByte(&OBC, &EUSCI_A_SPI_receiveData, &masterReceiveBufferPtr, masterReceiveBuffer);
                }

                setMasterListenState(requestedMasterListenState);

                break;

            // Transmit data case
            case USCI_SPI_UCTXIFG:
                setMasterListenState(requestedMasterListenState);

                // Transmit data to master
                sendByteToMaster();

                break;

            default:
                break;
        }
}
