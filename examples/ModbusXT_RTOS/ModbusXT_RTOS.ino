//You need to install NilRTOS library to make this example work
//NilRTOS library can be found in Greiman's github - https://github.com/greiman/NilRTOS-Arduino
//Notice that this example only works with AVR

#include "ModbusXT.h"
#include <NilSerial.h>
#include <NilRTOS.h>

#define Serial NilSerial

// Declare a semaphore with an inital counter value of zero.
SEMAPHORE_DECL(sem, 0);

#define TIMEOUT 500   //Timeout for a failed packet. Timeout need to larger than polling
#define POLLING 30     //Wait time to next request

#define BAUD        57600  
#define RETRIES     10    //How many time to re-request packet frome slave if request is failed
#define BYTE_FORMAT SERIAL_8E1
#define TxEnablePin 2   //Arduino pin to enable transmission

#define print(x)  Serial.print(x)
#define println(x) Serial.println(x)

//Name for register in regs[]
enum {
  button1,
  button2,
  button3,
  number_entry,
  password_entry,
  slider,
  total_packets,
  total_requests,
  total_failed,
  transfer_rate,
  transfer_delay,
  led_grn,
  led_blue,
  led_red,
  graph,
  TOTAL_REGS,
};

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum {
  PACKET1,
  PACKET2,
  NO_OF_PACKET
};

uint16_t regs[TOTAL_REGS];

//Modbus packet
Packet packets[NO_OF_PACKET];

// Access individual packet parameter. Uncomment it if you know what you're doing
// packetPointer packet1 = &packets[PACKET1];
// packetPointer packet2 = &packets[PACKET2];

const uint8_t hmiID = 1;  //HMI slave ID

long sm,em,dm;
int graph_value = 0;
int slider_value = 0;
uint16_t d=0;
uint8_t e=1;
const uint16_t password = 12345;
uint16_t num, temp;

//Modbus class define
Modbus master;

//Modbus communication in this thread
NIL_WORKING_AREA(waThread1, 100);
// Declare the thread function for thread 1.
NIL_THREAD(Thread1, arg) {
  bool ready_flag = 1;
  bool packet_ok = 0;
  systime_t prev_time = nilTimeNow();
  while (1) {

    //Request packet
    if (ready_flag)
    {
      master.request();
      ready_flag = 0;
      prev_time = nilTimeNow();
    }

    //check polling
    uint8_t finish=(nilTimeNow()-prev_time) > POLLING;
    if (packet_ok && finish)
    {
      ready_flag = 1;
      packet_ok = 0;
      prev_time = nilTimeNow();
    }
    else if (packet_ok && !finish)
      nilThdSleep(POLLING - (nilTimeNow() - prev_time - 1));

    //check response
    if (!packet_ok)
    {
      packet_ok = master.response();
      if ( (nilTimeNow() - prev_time ) > TIMEOUT  ) //check timeout
      {
        println("timeout");
        packet_ok = 1;
        master.error();
        prev_time = nilTimeNow();
      }
      
    }  

  }//end of while
}//end of thread 1

//This thread is using to calculate transfer rate
NIL_WORKING_AREA(waThread2, 20);
// Declare the thread function for thread 1.
NIL_THREAD(Thread2, arg) {
  while(1)
  {
    nilThdSleep(1000);
    regs[transfer_rate] = regs[total_requests] - temp;
    regs[transfer_delay] = (unsigned int) ((NO_OF_PACKET*100000UL)/regs[transfer_rate]);
    temp = regs[total_requests];
  }
}
//-----------NilRTOS--------------
NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY("thread1", Thread1, NULL, waThread1, sizeof(waThread1))
NIL_THREADS_TABLE_ENTRY("thread2", Thread2, NULL, waThread2, sizeof(waThread2))
NIL_THREADS_TABLE_END()
//--------------------------------

void setup()
{
  //Config packets and register
  master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 0, 6, 0);

  master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 100, 9, 6);

  //Start Modbus
  master.begin(&Serial1, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin);

  //debug on serial0
  Serial.begin(57600);  

  println("Arduino Modbus Master");

  //Arduino LED
  pinMode(13, OUTPUT);

  //Start RTOS
  nilSysBegin();
}

void loop()
{
  sm = millis();

  graph_value++;  //update graph data, just increase from -32768 to 32767 (signed int)

  regs[total_packets] = NO_OF_PACKET;             //Total number of packet, here is 2
  regs[total_requests] = master.total_requests(); //Update all requested packets. Take a look on ModbusXT.h
  regs[total_failed] = master.total_failed();     //Update all failed packet
  regs[graph] = graph_value;  //Update graph value

  //If button is press, turn on HMI's LED
  for (uint8_t i=0;i<3;i++)
  {
    if (regs[i] == 1)      
      regs[i+11] = 1;
    else
      regs[i+11] = 0;
  }

  //If Led green is on (or button 0 = 1) turn on Led on arduino
  if (regs[led_grn])
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);

  //Print number value on HMI
  if (num != regs[number_entry] )
  {
    num = regs[number_entry];
    print("Number: ");
    println(num);
  }

  //Print slider value on HMI
  if (slider_value != regs[slider] )
  {
    slider_value = regs[slider];
    print("Slider: ");
    println(slider_value);
  }

}//end loop