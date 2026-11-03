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
    void configureDeviceIdentification(const __FlashStringHelper *vendorName, const __FlashStringHelper *productCode, const __FlashStringHelper *version);
    void setVendorUrl(const __FlashStringHelper *vendorUrl);
    void setProductName(const __FlashStringHelper *productName);
    void setModelName(const __FlashStringHelper *modelName);
    void setApplicationName(const __FlashStringHelper *applicationName);
    void processPdu(ModbusADU& adu);

  private:
    // Values are the Modbus object IDs.
    enum : uint8_t {
      VENDOR_NAME,
      PRODUCT_CODE,
      MAJOR_MINOR_REVISION,
      VENDOR_URL,
      PRODUCT_NAME,
      MODEL_NAME,
      USER_APPLICATION_NAME,
      NUM_DEVICE_OBJECTS
    };
    static const uint16_t MAX_DATA_LEN = 252;
    static const uint16_t MAX_OBJECT_LEN = MAX_DATA_LEN - 6 - 2; // less the response header, object id and object length

    bool *_coils = 0;
    bool *_discreteInputs = 0;
    uint16_t *_holdingRegisters = 0;
    uint16_t *_inputRegisters = 0;
    uint16_t _numCoils = 0;
    uint16_t _numDiscreteInputs = 0;
    uint16_t _numHoldingRegisters = 0;
    uint16_t _numInputRegisters = 0;
    const __FlashStringHelper *_deviceObjects[NUM_DEVICE_OBJECTS] = {};

    // Only configureDeviceIdentification() sets this, so that sketches that never call it don't link the code behind it.
    void (*_deviceIdentificationHandler)(ModbusSlaveLogic& logic, ModbusADU& adu) = 0;

    void _processReadCoils(ModbusADU& adu);
    void _processReadDiscreteInputs(ModbusADU& adu);
    void _processReadHoldingRegisters(ModbusADU& adu);
    void _processReadInputRegisters(ModbusADU& adu);
    void _processWriteSingleCoil(ModbusADU& adu);
    void _processWriteSingleHoldingRegister(ModbusADU& adu);
    void _processWriteMultipleCoils(ModbusADU& adu);
    void _processWriteMultipleHoldingRegisters(ModbusADU& adu);
    void _processReadDeviceIdentification(ModbusADU& adu);

    static void _handleDeviceIdentification(ModbusSlaveLogic& logic, ModbusADU& adu);
    bool _addDeviceObject(uint8_t *data, uint16_t& dataLen, uint8_t objectId);
    void _processReadValues(ModbusADU& adu, bool buf[], uint16_t bufSize);
    void _processReadValues(ModbusADU& adu, uint16_t buf[], uint16_t bufSize);

};

#endif