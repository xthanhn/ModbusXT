Author: caddish12

ModbusXT changes:
- v1: work with polling
- v2: change Modbus class to individual - Modbus hmi;
+ change Modbus::Modbus to void Modbus::begin(port,baud,config,timeout,poling,tx)
+ add packetPointer to access individual packet
+ add process_F1_F2
- v3: add packetError & packetSuccess
+ Add timeout
+ add polling time: work with 5ms
- v4:
+ example: multiple packets
- v5:
+ complete example with polling and rtos
+ Almost complete library

Readme:
- With Arduino DUE, Serial0 will present an error