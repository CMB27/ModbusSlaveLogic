#ifndef ModbusSlaveLogic_h
#define ModbusSlaveLogic_h

#include "Arduino.h"
#include "ModbusADU.h"

class ModbusSlaveLogic {
  public:
    void configureCoils(bool coils[], uint16_t numCoils);
    void configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs);
    void configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters);
    void configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters);
    uint8_t getFunctionCode();
    uint16_t getDataAddress();
    uint16_t getDataQuantity();
    uint8_t getExceptionResponse();

  protected:
    void processPdu(ModbusADU& adu);
    void clearDebugValues();

  private:
    bool *_coils = 0;
    bool *_discreteInputs = 0;
    uint16_t *_holdingRegisters = 0;
    uint16_t *_inputRegisters = 0;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    uint8_t _exceptionResponse = 0;
    uint8_t _functionCode = 0;
    uint16_t _address = 0;
    uint16_t _quantity = 0;

    void _processReadCoils(ModbusADU& adu);
    void _processReadDiscreteInputs(ModbusADU& adu);
    void _processReadHoldingRegisters(ModbusADU& adu);
    void _processReadInputRegisters(ModbusADU& adu);
    void _processWriteSingleCoil(ModbusADU& adu);
    void _processWriteSingleHoldingRegister(ModbusADU& adu);
    void _processWriteMultipleCoils(ModbusADU& adu);
    void _processWriteMultipleHoldingRegisters(ModbusADU& adu);

    void _processReadValues(ModbusADU& adu, bool buf[], uint16_t bufSize);
    void _processReadValues(ModbusADU& adu, uint16_t buf[], uint16_t bufSize);

    void _prepareExceptionResponse(ModbusADU& adu, uint8_t exceptionCode);

};

#endif