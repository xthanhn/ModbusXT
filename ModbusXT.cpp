/*
Name: Arduino Modbus RTU Master
Date: 19/07/2015

This library is based on SimpleModbus of Mr.Juan
if you have any question, please feel free to email me at mr.caddish@gmail.com
*/
#include "ModbusXT.h"

#define IDLE 1
#define WAITING_FOR_REPLY 2
#define WAITING_FOR_TURNAROUND 3

#define ILLEGAL_FUNCTION 1
#define ILLEGAL_DATA_ADDRESS 2
#define ILLEGAL_DATA_VALUE 3


#define DEBUG_UART 0

#define DEBUG_HMI 1


#if DEBUG_UART
#define print(x) Serial.print(x)
#define println(x) Serial.println(x)
#else
	#define print(x)
	#define println(x)
#endif

//-----------------------------------------------------------------------------------
/* Request packet from slaver
 * @param: none
 * @return: status of returned packet. 
 * 		0 - No packet returned
 * 		1 - Packet error
 * 		2 - packet received successfully
 * @api
 */
uint8_t Modbus::response()
{
	
	checkPacket();

	if (_response_flag)
	{
		_response_flag = false;
		return 1;	//if have any response
	}
	else
		return 0;

}

//-----------------------------------------------------------------------------------
/* Manual request packet from slaver
 * @param: none
 * @return: none
 * @api
 */
void Modbus::request()
{
	static uint8_t packet_index = 0;
	unsigned int failed_connections = 0;
	
	unsigned char current_connection;

	// if (packet_index == _total_packets )
	// 	packet_index = 0;
	// _packet = &_packet_array[packet_index];
	// packet_index++;
	// constructPacket();

	do
	{		
		if (packet_index == _total_packets) // wrap around to the beginning
			packet_index = 0;
	
		// proceed to the next packet
		_packet = &_packet_array[packet_index];
	
		// get the current connection status
		current_connection = _packet->connection;
	
		if (!current_connection)
		{		
			// If all the connection attributes are false return
			// immediately to the main sketch
			if (++failed_connections == _total_packets)
				return;
		}
	
		packet_index++;
		
	} while (!current_connection); // while a packet has no connection get the next one
	constructPacket();
}

void Modbus::update()
{
	if (_transmission_ready_flag)
	{
		static uint8_t packet_index = 0;
		unsigned int failed_connections = 0;
	
		unsigned char current_connection;
	
		do
		{		
			if (packet_index == _total_packets) // wrap around to the beginning
				packet_index = 0;
		
			// proceed to the next packet
			_packet = &_packet_array[packet_index];
		
			// get the current connection status
			current_connection = _packet->connection;
		
			if (!current_connection)
			{		
				// If all the connection attributes are false return
				// immediately to the main sketch
				if (++failed_connections == _total_packets)
					return;
			}
		
			packet_index++;
			
		} while (!current_connection); // while a packet has no connection get the next one
		
		constructPacket();
	}

	//check response packet
	checkPacket();

	status();
}

void Modbus::status()
{
	unsigned char pollingFinished = (millis() - _delayStart) > _polling;

	if (_response_flag && pollingFinished ) //if slave responsed error or success
	{
		_response_flag = false;
		_transmission_ready_flag = true;
	}
}

//-----------------------------------------------------------------------------------
/* Request Check validity of packet and process the result
 * @param: none
 * @return: none
 * @private
 */
void Modbus::checkPacket()
{
	//Check packet response
	uint8_t buffer = getPacket();
	//if there is nothing received -> return
	if ( buffer == 0 )
		return;

#if DEBUG_UART
	print(F("Response: "));
	for (uint8_t i=0;i<buffer;i++)
	{
		print(frame[i]);
	}
	print('\t');
#endif

	uint8_t stt = 0;

	//Check exception response. Slave with OR with 0x80 if exception exists
	if ( (frame[1] & 0x80) == 0x80 )
	{	
		println(F("Packet errors"));
		switch(frame[2]) //3rd is the exception code
		{
			case ILLEGAL_FUNCTION:
				println(F("Illegal function or not support"));
				break;
			case ILLEGAL_DATA_VALUE:
				println(F("Illegal Value"));
				break;
			case ILLEGAL_DATA_ADDRESS:
				println(F("Illegal data address"));
				break;
			default:
				println(F("Misc exception"));
		}
		packetError();
		return;
	}//check exception

	uint16_t received_crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]); 
	uint16_t calculated_crc = calculateCRC(buffer - 2);

	if ( calculated_crc == received_crc )	//verify checksum
	{
		#if DEBUG_UART
		print("CRC! ");
		#endif
		//Check packet functions
		switch( frame[1] )
		{
			case READ_COIL_STATUS:
	        case READ_INPUT_STATUS:
	        	process_F1_F2();
	        	break;
	        case READ_INPUT_REGISTERS:
	        case READ_HOLDING_REGISTERS:
	        	process_F3_F4();
	        	break;
			case FORCE_SINGLE_COIL:
			case PRESET_SINGLE_REGISTER:
	        case FORCE_MULTIPLE_COILS:
	        case PRESET_MULTIPLE_REGISTERS:
	        	process_F5_F6_F15_F16();
	        	break;
	        default: // illegal function returned
	        	packetError();
	        break;   
		}

		return;
	}//CRC check
	else
	{
		packetError();
		println(F("CRC errors"));
		return;
	}


}

//-----------------------------------------------------------------------------------
/* Request Process packet of function 1 and function 2
 * @param: none
 * @return: Holding register in 2 bytes
 * @private
 */
void Modbus::process_F1_F2()
{
	//Serial.println("process_F1_F2");
	// packet->data for function 1 & 2 is actually the number of boolean points
  unsigned char no_of_registers = _packet->data / 16;
  unsigned char number_of_bytes = no_of_registers * 2; 
       
  // if the number of points dont fit in even 2byte amounts (one register) then use another register and pad 
  if (_packet->data % 16 > 0) 
  {
    no_of_registers++;
    number_of_bytes++;
  }
             
  if (frame[2] == number_of_bytes) // check number of bytes returned
  { 
    unsigned char bytes_processed = 0;
    unsigned char index = 3; // start at the 4th element in the frame and combine the Lo byte  
    unsigned int temp;
    for (unsigned char i = 0; i < no_of_registers; i++)
    {
      temp = frame[index]; 
      bytes_processed++;
      if (bytes_processed < number_of_bytes)
      {
				temp = (frame[index + 1] << 8) | temp;
        bytes_processed++;
        index += 2;
      }
      _register_array[_packet->register_start_address + i] = temp;
    }
    packetSuccess(); 
  }
  else // incorrect number of bytes returned 
  	packetError();
}

//-----------------------------------------------------------------------------------
/* Request Process packet of function 3 and function 4
 * @param: none
 * @return: Holding register in 2 bytes
 * @private
 */
void Modbus::process_F3_F4()
{
	//Frame[2] is number of bytes returned in uint16_t = 2 bytes
	if ( frame[2] == ( _packet->data * 2) )
	{
		uint8_t index = 3; //3nd bytes
		for (uint8_t i=0;i < _packet->data; i++ )
		{
			_register_array[i] = ( frame[index] << 8) | frame[index+1];
			index += 2;	//increase 2 bytes
		}
		packetSuccess();
	}
	else
		packetError();
}

//-----------------------------------------------------------------------------------
/* Request Process packet of function 5,6,15,16
 * @param: none
 * @return: none
 * @private
 * @comment: just confirm received packet with requested packet
 */
void Modbus::process_F5_F6_F15_F16()
{
	//Serial.println("process_F5_F6");
	// The repsonse of functions 5,6,15 & 16 are just an echo of the query
  unsigned int recieved_address = ((frame[2] << 8) | frame[3]);
  unsigned int recieved_data = ((frame[4] << 8) | frame[5]);
		
  if ((recieved_address == _packet->address) && (recieved_data == _packet->data))
    packetSuccess();
  else
    packetError();
}

//-----------------------------------------------------------------------------------
/* Request report successful packets
 * @param: none
 * @return: none
 * @private
 * @comment: just confirm received packet with requested packet
 */
void Modbus::packetSuccess()
{
	_packet->successful_requests++;
	_packet->retries = 0;
	_response_flag = true;	//got response
	_delayStart = millis();
}

//-----------------------------------------------------------------------------------
/* Request report for failed packets
 * @param: none
 * @return: none
 * @private
 * @comment: just confirm received packet with requested packet
 */
void Modbus::packetError()
{
	_response_flag = true;	//got response

	_packet->retries++;
	_packet->failed_requests++;
	_total_fail++;
	
	// if the number of retries have reached the max number of retries 
    // allowable, stop requesting the specific packet
    if (_packet->retries == _retry_count)
	{
		println("Max retry");
    	_packet->connection = 0;
		_packet->retries = 0;
	}
	_delayStart = millis();
	
}
//-----------------------------------------------------------------------------------
/* Request Cunstruct packet to send
 * @param: none
 * @return: none
 * @private
 * @comment: none
 */
void Modbus::constructPacket()
{
	//Disable next transmission until get response packet or timeout
	_transmission_ready_flag = false;

	_packet->requests++;
	_total_request++;	//calculate total packets are requested

	//Modbus Application Protocol v1.13b
	frame[0] = _packet->id;
	frame[1] = _packet->function;
	frame[2] = _packet->address >> 8; //Address Hi
	frame[3] = _packet->address & 0xFF; //Address Lo

	//If packet is single regiser, data is what it is
	if ( _packet->function == PRESET_SINGLE_REGISTER ){
		_packet->data = _register_array[_packet->register_start_address];
	}

	//2 bytes address
	frame[4] = _packet->data >> 8; 	//total registers Hi
	frame[5] = _packet->data & 0xFF;	//total registers Lo

	//Frame size for function code 3, 4 & 6 = 8
	uint8_t frameSize;
	if (_packet->function == PRESET_MULTIPLE_REGISTERS) 
		frameSize = construct_F16();
	else if (_packet->function == FORCE_MULTIPLE_COILS)
		frameSize = construct_F15();
	else // else functions 1,2,3,4,5 & 6 is assumed. They all share the exact same request format.
    	frameSize = 8; // the request is always 8 bytes in size for the above mentioned functions.

	//Error check CRC16
	uint16_t crc16 = calculateCRC(frameSize - 2);	//First 6 bytes of frame is used to calculate CRC error check
	frame[frameSize - 2] = crc16 >> 8;	//High byte of CRC
	frame[frameSize - 1] = crc16 & 0xFF;	//Low byte of CRC

#if DEBUG_UART
	print("Request:  ");
	for (uint8_t i=0;i<frameSize;i++)
		print(frame[i]);
	println();
#endif

	//Send packet frame to slave
	sendPacket(frameSize);
}

//-----------------------------------------------------------------------------------
/* Forces each coil (0X reference) in a sequence of coils to either ON or OFF.
 * @param: none
 * @return: none
 * @private
 * @comment: none
 */
uint8_t Modbus::construct_F15()
{
	// function 15 coil information is packed LSB first until the first 16 bits are completed
  // It is received the same way..
  uint8_t no_of_registers = _packet->data / 16;
  uint8_t no_of_bytes = no_of_registers * 2; 
	
  // if the number of points dont fit in even 2byte amounts (one register) then use another register and pad 
  if (_packet->data % 16 > 0) 
  {
    no_of_registers++;
    no_of_bytes++;
  }
	
  frame[6] = no_of_bytes;
  uint8_t bytes_processed = 0;
  uint8_t index = 7; // user data starts at index 7
	uint16_t temp;
	
  for (uint8_t i = 0; i < no_of_registers; i++)
  {
    temp = _register_array[_packet->register_start_address + i]; // get the data
    frame[index] = temp & 0xFF; 
    bytes_processed++;
     
    if (bytes_processed < no_of_bytes)
    {
      frame[index + 1] = temp >> 8;
      bytes_processed++;
      index += 2;
    }
  }
	uint8_t frameSize = (9 + no_of_bytes); // first 7 bytes of the array + 2 bytes CRC + noOfBytes 
	return frameSize;
}

//-----------------------------------------------------------------------------------
/* Presets values into a sequence of holding registers (4X references).
 * @param: none
 * @return: size of packet
 * @private
 * @comment: none
 */
uint8_t Modbus::construct_F16()
{
	uint8_t no_of_bytes = _packet->data * 2; 
    
	// first 6 bytes of the array + no_of_bytes + 2 bytes CRC 
	frame[6] = no_of_bytes; // number of bytes
	uint8_t index = 7; // user data starts at index 7
	uint8_t no_of_registers = _packet->data;
	uint16_t temp;
		
  	for (uint8_t i = 0; i < no_of_registers; i++)
  	{
	    temp = _register_array[_packet->register_start_address + i]; // get the data
	    frame[index] = temp >> 8;
	    index++;
	    frame[index] = temp & 0xFF;
	    index++;
  	}
	uint8_t frameSize = (9 + no_of_bytes); // first 7 bytes of the array + 2 bytes CRC + noOfBytes 
	return frameSize;
}

//-----------------------------------------------------------------------------------
/* Intitialize Modbus config
 * @param: Modbus config
 * 		- Mobus port
 *		- baud: baurate of transmission
 * 		- timeout: time to wait for returned packet
 * 		- polling:	time between two packet
 *		- retry: how many time packet is resent if it was failed
 *		- TxEnablePin: for RS485 only, pin to enable transmission
 * @return: size of packet
 * @private
 * @comment: none
 */
#if defined(__SAM3X8E__)
	void Modbus::begin(USARTClass* modbusPort, long baud, uint8_t byteFormat,long timeout, long polling, uint8_t retry, uint8_t TxEnablePin)
#else
	void Modbus::begin(HardwareSerial* modbusPort, long baud, uint8_t byteFormat, long timeout, long polling, uint8_t retry, uint8_t TxEnablePin)
#endif
{

	/*
	Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
  	for inter character time out and 1.75 ms for a frame delay for baud rates
  	below 19200 the timing is more critical and has to be calculated.
  	E.g. 9600 baud in a 10 bit packet is 9600/10 (8E1) = 960 characters per second
  	In milliseconds this will be 960 characters per 1000ms. So for 1 character
  	1000ms/960 characters is 1.04167ms per character and finally modbus states
  	an inter-character must be 1.5T or 1.5 times longer than a character. Thus
  	1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.
	Thus the formula is T1.5(us) = (1000ms * 1000(us) * 1.5 * 10bits)/baud
	1000ms * 1000(us) * 1.5 * 10bits = 15000000 can be calculated as a constant
	*/

	_baud = baud;
	if (_baud > 19200)
		T1_5 = 750;	
	else
		T1_5 = 15000000/_baud; // 1T * 1.5 = T1.5

	/* 
	The modbus definition of a frame delay is a waiting period of 3.5 character times
	between packets. This is not quite the same as the frameDelay implemented in
	this library but does benifit from it.
	The frameDelay variable is mainly used to ensure that the last character is 
	transmitted without truncation. A value of 2 character times is chosen which
	should suffice without holding the bus line high for too long.
	*/
	
	_frame_delay = T1_5 * 2;

	_modbusPort = modbusPort;

	_byteFormat = byteFormat;
	_timeout = timeout;
	_polling = polling;
	_retry_count = retry;
	_TxEnablePin = TxEnablePin;
#if defined(__SAM3X8E__)
	(*_modbusPort).begin(_baud,SERIAL_8E1);
#else
	(*_modbusPort).begin(_baud,_byteFormat);
#endif

	TxEnableConfig();

	_transmission_ready_flag = true; //start 1st time
}

//-----------------------------------------------------------------------------------
/* Intitialize packet param
 * @param: Packets to send and register of master to hold data
 *		- packets: packet pointer
 *		- total_packets: number of total packet
 *		- register_array: array of register to hold data to send or receive from slaver
 * @return: size of packet
 * @private
 * @comment: none
 */
void Modbus::configure(Packet* packets, uint16_t total_packets, uint16_t* register_array)
{
	_packet_array = packets;
	_register_array = register_array;
	_total_packets = total_packets;
	_manual_request = false;
}

//-----------------------------------------------------------------------------------
/* Intitialize packet param
 * @param: Packets to send and register of master to hold data
 *		- packets: packet pointer
 *		- total_packets: number of total packet
 *		- register_array: array of register to hold data to send or receive from slaver
 * @return: size of packet
 * @private
 * @comment: none
 */
void Modbus::configure_manual(Packet* packets, uint16_t total_packets, uint16_t* register_array)
{
	_packet_array = packets;
	_register_array = register_array;
	_total_packets = total_packets;
	_manual_request = true;
}

//-----------------------------------------------------------------------------------
/* Config individual packet param
 * @param: Packets to send and register of master to hold data
 *		- packets: packet pointer
 *		- total_packets: number of total packet
 *		- register_array: array of register to hold data to send or receive from slaver
 * @return: size of packet
 * @private
 * @comment: none
 */
void Modbus::construct(Packet *packet, 
										uint8_t id,
										uint8_t function,
										uint16_t address,
										uint16_t data,
										uint16_t register_start_address)
{
	packet->id = id;
	packet->function  = function;
	packet->address = address;
	packet->data = data;
	packet->register_start_address = register_start_address;
	packet->connection = 1;
}



//-----------------------------------------------------------------------------------
/* 16 bit checksum
 * @param: Packet frame's size
 * @return: CRC16
 * @private
 * @comment: none
 */
uint16_t Modbus::calculateCRC(uint8_t frame_size) 
{
  uint16_t temp, temp2, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < frame_size; i++)
  {
    temp = temp ^ frame[i];
    for (uint8_t j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp; 
}


//-----------------------------------------------------------------------------------
/* Send packet
 * @param: frame's size
 * @return: none
 * @private
 * @comment: none
 */
void Modbus::sendPacket(uint8_t bufferSize)
{
	TxEnable();	//Enable transmittion
		
	for (uint8_t i = 0; i < bufferSize; i++)
	{
		// #if DEBUG_UART
		// Serial.print(frame[i]);
		// #endif
		(*_modbusPort).write(frame[i]);
	}
	// #if DEBUG_UART
	// Serial.println();
	// #endif
		
	(*_modbusPort).flush();
	
	delayMicroseconds(_frame_delay);
	
	TxDisable();	//Disable transmittion
		
	_delayStart = millis(); // start the timeout delay	
}


//-----------------------------------------------------------------------------------
/* Send packet
 * @param: none
 * @return: received packet's size
 * @private
 * @comment: none
 */
uint8_t Modbus::getPacket()
{
	// if (!_wait_response_flag)
	// 	return 0;
	if ( ((*_modbusPort).available() > 0)  )
	{
		uint8_t overflowFlag = 0;
		uint8_t buffer = 0;
		while((*_modbusPort).available())
		{
			/*
			The maximum number of bytes is limited to the serial buffer size 
	      	of BUFFER_SIZE. If more bytes is received than the BUFFER_SIZE the 
	      	overflow flag will be set and the serial buffer will be read until
	      	all the data is cleared from the receive buffer, while the slave is 
	      	still responding.
	      	*/
			if (overflowFlag)
			{
				println(F("Overflow"));
				(*_modbusPort).read();
			}
			else
			{
				if (buffer == BUFFER_SIZE)
					overflowFlag = 1;
				frame[buffer] = (*_modbusPort).read();
				buffer++;
			}
			
			/*
			This is not 100% correct but it will suffice.
			worst case scenario is if more than one character time expires
			while reading from the buffer then the buffer is most likely empty
			If there are more bytes after such a delay it is not supposed to
			be received and thus will force a frame_error.
			*/
			delayMicroseconds(T1_5); //character time wait
		}

		/*
		The minimum buffer size from a slave can be an exception response of
    	5 bytes. If the buffer was partially filled set a frame_error.
		The maximum number of bytes in a modbus packet is 256 bytes.
		The serial buffer limits this to 64 bytes.
		*/
		if ( buffer < 5 || overflowFlag )
		{
			//println(F("Buffer errors"));
			buffer = 0;
		}
		else if ( frame[0] != _packet->id )  //return if ID returned is not matched
		{
			//println(F("ID not matched"));
			buffer = 0;
		}
#if DEBUG_HMI
		_response_time = millis() - _delayStart;	//return the response time
#endif
		return buffer;
	} //serial available
	else
	if (!_manual_request)
	{
		if (((millis() - _delayStart) > _timeout ) && !_transmission_ready_flag )
		{
			println("timeout");
			_transmission_ready_flag = true;	//allow to start new transmission
			packetError();
			
		}
	}
	
	return 0;
}//end of getData



//-----------------------------------------------------------------------------------
/* Modbus enable transmission
 * @param: none
 * @return: none
 * @private
 * @comment: none
 */
void Modbus::TxEnable()
{
	digitalWrite(_TxEnablePin, HIGH);
}

//-----------------------------------------------------------------------------------
/* Modbus disable transmission
 * @param: none
 * @return: none
 * @private
 * @comment: none
 */
void Modbus::TxDisable()
{
	digitalWrite(_TxEnablePin, LOW);
}

//-----------------------------------------------------------------------------------
/* Modbus TxEnable pin config
 * @param: none
 * @return: none
 * @private
 * @comment: none
 */
void Modbus::TxEnableConfig()
{
	//Setup Tx enable pin
	pinMode(_TxEnablePin,OUTPUT);
	digitalWrite(_TxEnablePin, LOW);
}





