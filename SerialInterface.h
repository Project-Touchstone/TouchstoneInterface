#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H


// Byte signifying end of data frame
#define END 0x0

class SerialInterface {
    private:
        // Current header
        static uint8_t header;
        // Whether new header has been received
        static bool headerFlag;
        // Whether current data frame has ended
        static bool endFlag;
    public:
        // Initializes the serial interface
        static void begin(long baudRate);

        // Checks for an incoming header or end byte
        static bool processHeader();

        // Gets the current header
        static uint8_t getHeader();

        // Sends a byte of data
        static void sendByte(uint8_t data);

        // Sends a data type in binary form
        template <typename T>
        static void sendData(T data);

        // Sends the end of data frame
        static void sendEnd();

        // Checks if the header has ended
        static bool isEnded();

        // Reads a byte of data
        static uint8_t readByte();

        // Reads a 32-bit float
        static float readFloat();

        // Clears the current header
        static void clearHeader();
};

template <typename T>
void SerialInterface::sendData(T data) {
    uint8_t buffer[sizeof(data)];
    for (int i = 0; i < sizeof(data); i++) {
        buffer[i] = (data & 0xFF);
        data = data >> 8;
    }
    Serial.write(buffer, sizeof(data));
}

#endif