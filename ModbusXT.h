/*
Name: Arduino Modbus RTU Master
Date: 19/07/2015

This library is based on SimpleModbus of Mr.Juan
if you have any question, please feel free to email me at mr.caddish@gmail.com
*/

#ifndef MODBUSXT_H_
#define MODBUSXT_H_


#include <Arduino.h>


#define BUFFER_SIZE 64

#define COIL_OFF 0x0000 // Function 5 OFF request is 0x0000
#define COIL_ON 0xFF00 // Function 5 ON request is 0xFF00
#define READ_COIL_STATUS 1 // Reads the ON/OFF status of discrete outputs (0X references, coils) in the slave.
#define READ_INPUT_STATUS 2 // Reads the ON/OFF status of discrete inputs (1X references) in the slave.
#define READ_HOLDING_REGISTERS 3 // Reads the binary contents of holding registers (4X references) in the slave.
#define READ_INPUT_REGISTERS 4 // Reads the binary contents of input registers (3X references) in the slave. Not writable.
#define FORCE_SINGLE_COIL 5 // Forces a single coil (0X reference) to either ON (0xFF00) or OFF (0x0000).
#define PRESET_SINGLE_REGISTER 6 // Presets a value into a single holding register (4X reference).
#define FORCE_MULTIPLE_COILS 15 // Forces each coil (0X reference) in a sequence of coils to either ON or OFF.
#define PRESET_MULTIPLE_REGISTERS 16 // Presets values into a sequence of holding registers (4X references).

typedef struct {
    //Packet unique info
    uint8_t     id;
    uint8_t     function;
    uint16_t    address;  
    uint16_t    data; //data
    /* 
    For functions 1 & 2 data is the number of points
    For function 5 data is either ON (oxFF00) or OFF (0x0000)
    For function 6 data is exactly that, one register's data
    For functions 3, 4 & 16 data is the number of registers to read
    For function 15 data is the number of coils to write
    */
    
    uint16_t    register_start_address;    //start register of Master to write or read from slave.

    //Modbus information
    uint16_t    requests;
    uint16_t    successful_requests;
    uint16_t    failed_requests;
    uint16_t    exception_errors;
    uint16_t    retries;

    //Packet connection status
    uint8_t connection;
} Packet;

typedef Packet* packetPointer;

class  Modbus {
    public:
        
        //-----------------------------------------------------------------------------------
        /* Config Modbus parameter and Initialize Modbus protocol
         * @param: none
         * @return: none
         * @api
         * @comment: none
         */
#if defined(__SAM3X8E__)
        void begin(USARTClass* modbusPort, long baud, uint8_t byteFormat, long timeout, long polling, uint8_t retry, uint8_t TxEnablePin);
#else
        void begin(HardwareSerial* modbusPort, long baud, uint8_t byteFormat, long timeout, long polling, uint8_t retry, uint8_t _TxEnablePin);
#endif
        
        //-----------------------------------------------------------------------------------
        /* Configure Modbus packets
         * @param: none
         * @return: none
         * @api
         * @comment: none
         */
        void configure(Packet *packets, uint16_t total_packet, uint16_t* register_array);  

        //-----------------------------------------------------------------------------------
        /* Configure Modbus packets - Modbus will not auto update, you need to call request and response manually
         * @param: none
         * @return: none
         * @api
         * @comment: none
         */
        void configure_manual(Packet *packets, uint16_t total_packet, uint16_t* register_array); 

        //-----------------------------------------------------------------------------------
        /* Construct individual packet with automatic start requesting packet
         * @param: none
         * @return: none
         * @api
         * @comment: none
         */
        void construct(Packet *packets, 
                                        uint8_t id,
                                        uint8_t function,
                                        uint16_t address,
                                        uint16_t data,
                                        uint16_t register_start_address); 

        //-----------------------------------------------------------------------------------
        /* Modbus packet data returned
         * @param: none
         * @return: 1 if packet is responsed correctly
         * @api
         * @comment: none
         */
        uint8_t response();

        //-----------------------------------------------------------------------------------
        /* Request to send modbus packets
         * @param: none
         * @return: none
         * @api
         * @comment: none
         */
        void request();

        //-----------------------------------------------------------------------------------
        /* Auto update modbus packets with polling and timeout
         * @param: none
         * @return: none
         * @api
         * @comment: none
         */
        void update();

        //-----------------------------------------------------------------------------------
        /* Return status of modbus transmission
         * @param: none
         * @return: 
         *      - 0: transmission is not ready
         *      - 1: transmission is ready
         * @api
         * @comment: none
         */
        bool ready() { return _transmission_ready_flag;}

        //-----------------------------------------------------------------------------------
        /* Return response time when packet is sent until receive response packet
         * @param: none
         * @return: time in milisecond
         * @api
         * @comment: only valid in debug mode
         */
        bool response_time() { 
            return _response_time;
        }

        //-----------------------------------------------------------------------------------
        /* Set packet error parameter
         * @param: none
         * @return: time in milisecond
         * @api
         * @comment:
         */
        void error()
        {
            packetError();
        }

        //-----------------------------------------------------------------------------------
        /* Return total requested packets
         * @param: none
         * @return: time in milisecond
         * @api
         * @comment: 
         */
        uint16_t total_requests()
        {
            return _total_request;
        }

        //-----------------------------------------------------------------------------------
        /* Return total packets failed
         * @param: none
         * @return: time in milisecond
         * @api
         * @comment: 
         */
        uint16_t total_failed()
        {
            return _total_fail;
        }
        
    private:
        //Local function

        //Check received packet
        void checkPacket();

        //Get response packet
        uint8_t getPacket();

        //Calculate checksum
        uint16_t calculateCRC(uint8_t frame_size);

        //Construct frame to send
        void constructPacket();

        //Construct frame for function 15
        uint8_t construct_F15();

        //Construct frame for function 16
        uint8_t construct_F16();

        //Send modbus packet
        void sendPacket(uint8_t bufferSize);

        //Enable transmission
        void TxEnable();

        //Disable transmisson
        void TxDisable();

        //Configure transmission pin
        void TxEnableConfig();

        //Process result for function 1 and 2
        void process_F1_F2();

        //Process result for function 3,4
        void process_F3_F4();

        //Process result for function 5,6,15,16
        void process_F5_F6_F15_F16();

        //Update packet error information
        void packetError();

        //Update packet success information
        void packetSuccess();

        //Measure polling time in auto update mode
        void status();
        
        
        long _response_time = 0;
        
        uint8_t wait = 0;

        long _baud;
        uint8_t _byteFormat;
        long _timeout;
        long _polling;
        uint8_t _retry_count;
        uint8_t _TxEnablePin;  

        long _delayStart; //measure timeout of returned packet

        uint16_t count=0;

        bool _manual_request = false;   //request by rtos

#if defined(__SAM3X8E__)
        USARTClass* _modbusPort;    //Serial for ARM - it will not work with Seria0 
#else
        HardwareSerial* _modbusPort;    //Serial for AVR
#endif

        Packet* _packet;    //current packet

        uint8_t frame[BUFFER_SIZE]; //frame of packet

        uint16_t T1_5;          //1.5 times of a character connection time
        uint16_t _frame_delay;   //delay time for frame

        uint16_t _total_packets;    //Total number of packets
        uint16_t* _register_array;  //registers that hold the dater or address of modbus register
        Packet* _packet_array;      //All initial packet   

        bool _transmission_ready_flag = false;  //=1 when transimission is not busy

        bool _response_flag = false;    //status of slave response = 1 or not = 0

        uint16_t _total_request;    //Total packets have requested
        uint16_t _total_fail;   //Total failed packets
};

#endif  //end if define

