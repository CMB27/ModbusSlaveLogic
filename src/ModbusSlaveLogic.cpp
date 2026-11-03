#include "ModbusSlaveLogic.h"

void ModbusSlaveLogic::configureCoils(bool coils[], uint16_t numCoils) {
  _coils = coils;
  _numCoils = numCoils;
}

void ModbusSlaveLogic::configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs) {
  _discreteInputs = discreteInputs;
  _numDiscreteInputs = numDiscreteInputs;
}

void ModbusSlaveLogic::configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters) {
  _holdingRegisters = holdingRegisters;
  _numHoldingRegisters = numHoldingRegisters;
}

void ModbusSlaveLogic::configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters) {
  _inputRegisters = inputRegisters;
  _numInputRegisters = numInputRegisters;
}

void ModbusSlaveLogic::configureDeviceIdentification(const __FlashStringHelper *vendorName, const __FlashStringHelper *productCode, const __FlashStringHelper *version) {
  _deviceObjects[VENDOR_NAME] = vendorName;
  _deviceObjects[PRODUCT_CODE] = productCode;
  _deviceObjects[MAJOR_MINOR_REVISION] = version;
  // This is a trick to allow the linker to drop the device identification code if it isn't configured
  _deviceIdentificationHandler = _handleDeviceIdentification;
}

void ModbusSlaveLogic::setVendorUrl(const __FlashStringHelper *vendorUrl) {
  _deviceObjects[VENDOR_URL] = vendorUrl;
}

void ModbusSlaveLogic::setProductName(const __FlashStringHelper *productName) {
  _deviceObjects[PRODUCT_NAME] = productName;
}

void ModbusSlaveLogic::setModelName(const __FlashStringHelper *modelName) {
  _deviceObjects[MODEL_NAME] = modelName;
}

void ModbusSlaveLogic::setApplicationName(const __FlashStringHelper *applicationName) {
  _deviceObjects[USER_APPLICATION_NAME] = applicationName;
}

void ModbusSlaveLogic::processPdu(ModbusADU& adu) {
  switch (adu.getFunctionCode()) {
    case 1:
      _processReadCoils(adu);
      break;
    case 2:
      _processReadDiscreteInputs(adu);
      break;
    case 3:
      _processReadHoldingRegisters(adu);
      break;
    case 4:
      _processReadInputRegisters(adu);
      break;
    case 5:
      _processWriteSingleCoil(adu);
      break;
    case 6:
      _processWriteSingleHoldingRegister(adu);
      break;
    case 15:
      _processWriteMultipleCoils(adu);
      break;
    case 16:
      _processWriteMultipleHoldingRegisters(adu);
      break;
    case 43:
      if (_deviceIdentificationHandler) _deviceIdentificationHandler(*this, adu);
      else adu.prepareExceptionResponse(1);
      break;
    default:
      adu.prepareExceptionResponse(1);
      break;
  }
}



void ModbusSlaveLogic::_processReadCoils(ModbusADU& adu) {
  _processReadValues(adu, _coils, _numCoils);
}

void ModbusSlaveLogic::_processReadDiscreteInputs(ModbusADU& adu) {
  _processReadValues(adu, _discreteInputs, _numDiscreteInputs);
}

void ModbusSlaveLogic::_processReadHoldingRegisters(ModbusADU& adu) {
  _processReadValues(adu, _holdingRegisters, _numHoldingRegisters);
}

void ModbusSlaveLogic::_processReadInputRegisters(ModbusADU& adu) {
  _processReadValues(adu, _inputRegisters, _numInputRegisters);
}

void ModbusSlaveLogic::_processWriteSingleCoil(ModbusADU& adu) {
  uint16_t address = adu.getDataRegister(0);
  uint16_t value = adu.getDataRegister(2);
  if (!_coils ||_numCoils == 0) adu.prepareExceptionResponse(1);
  else if (value != 0 && value != 0xFF00) adu.prepareExceptionResponse(3);
  else if (address >= _numCoils) adu.prepareExceptionResponse(2);
  else _coils[address] = value;
}

void ModbusSlaveLogic::_processWriteSingleHoldingRegister(ModbusADU& adu) {
  uint16_t address = adu.getDataRegister(0);
  uint16_t value = adu.getDataRegister(2);
  if (!_holdingRegisters || _numHoldingRegisters == 0) adu.prepareExceptionResponse(1);
  else if (address >= _numHoldingRegisters) adu.prepareExceptionResponse(2);
  else _holdingRegisters[address] = value;
}

void ModbusSlaveLogic::_processWriteMultipleCoils(ModbusADU& adu) {
  uint16_t startAddress = adu.getDataRegister(0);
  uint16_t quantity = adu.getDataRegister(2);
  if (!_coils || _numCoils == 0) adu.prepareExceptionResponse(1);
  else if (quantity == 0 || quantity > 1968 || adu.data[4] != div8RndUp(quantity)) adu.prepareExceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity)) adu.prepareExceptionResponse(2);
  else {
    for (uint16_t i = 0; i < quantity; i++) {
      _coils[startAddress + i] = bitRead(adu.data[5 + (i / 8)], i % 8);
    }
    adu.setDataLen(4);
  }
}

void ModbusSlaveLogic::_processWriteMultipleHoldingRegisters(ModbusADU& adu) {
  uint16_t startAddress = adu.getDataRegister(0);
  uint16_t quantity = adu.getDataRegister(2);
  if (!_holdingRegisters || _numHoldingRegisters == 0) adu.prepareExceptionResponse(1);
  else if (quantity == 0 || quantity > 123 || adu.data[4] != (quantity * 2)) adu.prepareExceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity)) adu.prepareExceptionResponse(2);
  else {
    for (uint16_t i = 0; i < quantity; i++) {
      _holdingRegisters[startAddress + i] = adu.getDataRegister(5 + (i * 2));
    }
    adu.setDataLen(4);
  }
}

void ModbusSlaveLogic::_processReadDeviceIdentification(ModbusADU& adu) {
  if (adu.getUnitId() == 0) return;
  uint8_t *data = adu.data;
  uint8_t meiType = data[0];
  uint8_t readCode = data[1];
  uint8_t objectId = data[2];
  bool objectExists = objectId < NUM_DEVICE_OBJECTS && _deviceObjects[objectId];
  if (meiType != 14 || !_deviceObjects[VENDOR_NAME]) adu.prepareExceptionResponse(1);
  else if (adu.getDataLen() != 3 || readCode < 1 || readCode > 4) adu.prepareExceptionResponse(3);
  else if (readCode == 4 && !objectExists) adu.prepareExceptionResponse(2);
  else {
    bool hasRegularObjects = false;
    for (uint8_t id = VENDOR_URL; id < NUM_DEVICE_OBJECTS; id++) {
      if (_deviceObjects[id]) hasRegularObjects = true;
    }
    data[2] = hasRegularObjects ? 0x82 : 0x81; // conformity level
    data[3] = 0; // more follows
    data[4] = 0; // next object id
    data[5] = 0; // number of objects
    uint16_t dataLen = 6;
    if (readCode == 4) _addDeviceObject(data, dataLen, objectId);
    else {
      uint8_t lastId = USER_APPLICATION_NAME;
      if (readCode == 1) lastId = MAJOR_MINOR_REVISION;
      uint8_t firstId = VENDOR_NAME;
      if (objectExists && objectId <= lastId) firstId = objectId;
      for (uint8_t id = firstId; id <= lastId; id++) {
        if (_deviceObjects[id] && !_addDeviceObject(data, dataLen, id)) {
          data[3] = 0xFF;
          data[4] = id;
          break;
        }
      }
    }
    adu.setDataLen(dataLen);
  }
}



void ModbusSlaveLogic::_handleDeviceIdentification(ModbusSlaveLogic& logic, ModbusADU& adu) {
  logic._processReadDeviceIdentification(adu);
}

bool ModbusSlaveLogic::_addDeviceObject(uint8_t *data, uint16_t& dataLen, uint8_t objectId) {
  PGM_P value = reinterpret_cast<PGM_P>(_deviceObjects[objectId]);
  uint16_t valueLen = strlen_P(value);
  if (valueLen > MAX_OBJECT_LEN) valueLen = MAX_OBJECT_LEN;
  if (dataLen + 2 + valueLen > MAX_DATA_LEN) return false;
  data[dataLen++] = objectId;
  data[dataLen++] = valueLen;
  memcpy_P(data + dataLen, value, valueLen);
  dataLen += valueLen;
  data[5]++; // number of objects
  return true;
}

void ModbusSlaveLogic::_processReadValues(ModbusADU& adu, bool buf[], uint16_t bufSize) {
  if (adu.getUnitId() == 0) return;
  uint16_t startAddress = adu.getDataRegister(0);
  uint16_t quantity = adu.getDataRegister(2);
  if (!buf || bufSize == 0) adu.prepareExceptionResponse(1);
  else if (quantity == 0 || quantity > 2000) adu.prepareExceptionResponse(3);
  else if (quantity > bufSize || startAddress > (bufSize - quantity)) adu.prepareExceptionResponse(2);
  else {
    uint8_t byteCount = div8RndUp(quantity);
    adu.data[0] = byteCount;
    for (uint16_t i = 0; i < (byteCount * 8); i++) {
      uint8_t byteIndex = 1 + (i / 8);
      uint8_t bitIndex = i % 8;
      if (i < quantity) bitWrite(adu.data[byteIndex], bitIndex, buf[startAddress + i]);
      else bitClear(adu.data[byteIndex], bitIndex);
    }
    adu.setDataLen(1 + byteCount);
  }
}

void ModbusSlaveLogic::_processReadValues(ModbusADU& adu, uint16_t buf[], uint16_t bufSize) {
  if (adu.getUnitId() == 0) return;
  uint16_t startAddress = adu.getDataRegister(0);
  uint16_t quantity = adu.getDataRegister(2);
  if (!buf || bufSize == 0) adu.prepareExceptionResponse(1);
  else if (quantity == 0 || quantity > 125) adu.prepareExceptionResponse(3);
  else if (quantity > bufSize || startAddress > (bufSize - quantity)) adu.prepareExceptionResponse(2);
  else {
    uint8_t byteCount = quantity * 2;
    adu.data[0] = byteCount;
    for (uint16_t i = 0; i < quantity; i++) {
      adu.setDataRegister(1 + (i * 2), buf[startAddress + i]);
    }
    adu.setDataLen(1 + byteCount);
  }
}