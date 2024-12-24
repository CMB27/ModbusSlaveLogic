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

uint8_t ModbusSlaveLogic::getFunctionCode() {
  return _functionCode;
}

uint16_t ModbusSlaveLogic::getDataAddress() {
  return _address;
}

uint16_t ModbusSlaveLogic::getDataQuantity() {
  return _quantity;
}

uint8_t ModbusSlaveLogic::getExceptionResponse() {
  return _exceptionResponse;
}



void ModbusSlaveLogic::processPdu(ModbusADU& adu) {
  _functionCode = adu.getFunctionCode();
  _address = 0;
  _quantity = 0;
  _exceptionResponse = 0;
  switch (_functionCode) {
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
    default:
      _prepareExceptionResponse(adu, 1);
      break;
  }
}

void ModbusSlaveLogic::clearDebugValues() {
  _functionCode = 0;
  _address = 0;
  _quantity = 0;
  _exceptionResponse = 0;
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
  _address = adu.getDataRegister(0);
  _quantity = 1;
  uint16_t value = adu.getDataRegister(2);
  if (!_coils || _numCoils == 0) _prepareExceptionResponse(adu, 1);
  else if (value != 0 && value != 0xFF00) _prepareExceptionResponse(adu, 3);
  else if (_address >= _numCoils) _prepareExceptionResponse(adu, 2);
  else _coils[_address] = value;
}

void ModbusSlaveLogic::_processWriteSingleHoldingRegister(ModbusADU& adu) {
  _address = adu.getDataRegister(0);
  _quantity = 1;
  uint16_t value = adu.getDataRegister(2);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _prepareExceptionResponse(adu, 1);
  else if (_address >= _numHoldingRegisters) _prepareExceptionResponse(adu, 2);
  else _holdingRegisters[_address] = value;
}

void ModbusSlaveLogic::_processWriteMultipleCoils(ModbusADU& adu) {
  _address = adu.getDataRegister(0);
  _quantity = adu.getDataRegister(2);
  if (!_coils || _numCoils == 0) _prepareExceptionResponse(adu, 1);
  else if (_quantity == 0 || _quantity > 1968 || adu.data[4] != div8RndUp(_quantity)) _prepareExceptionResponse(adu, 3);
  else if (_quantity > _numCoils || _address > (_numCoils - _quantity)) _prepareExceptionResponse(adu, 2);
  else {
    for (uint16_t i = 0; i < _quantity; i++) {
      _coils[_address + i] = bitRead(adu.data[5 + (i / 8)], i % 8);
    }
    adu.setDataLen(4);
  }
}

void ModbusSlaveLogic::_processWriteMultipleHoldingRegisters(ModbusADU& adu) {
  _address = adu.getDataRegister(0);
  _quantity = adu.getDataRegister(2);
  if (!_holdingRegisters || _numHoldingRegisters == 0) _prepareExceptionResponse(adu, 1);
  else if (_quantity == 0 || _quantity > 123 || adu.data[4] != (_quantity * 2)) _prepareExceptionResponse(adu, 3);
  else if (_quantity > _numHoldingRegisters || _address > (_numHoldingRegisters - _quantity)) _prepareExceptionResponse(adu, 2);
  else {
    for (uint16_t i = 0; i < _quantity; i++) {
      _holdingRegisters[_address + i] = adu.getDataRegister(5 + (i * 2));
    }
    adu.setDataLen(4);
  }
}



void ModbusSlaveLogic::_processReadValues(ModbusADU& adu, bool buf[], uint16_t bufSize) {
  if (adu.getUnitId() == 0) return;
  _address = adu.getDataRegister(0);
  _quantity = adu.getDataRegister(2);
  if (!buf || bufSize == 0) _prepareExceptionResponse(adu, 1);
  else if (_quantity == 0 || _quantity > 2000) _prepareExceptionResponse(adu, 3);
  else if (_quantity > bufSize || _address > (bufSize - _quantity)) _prepareExceptionResponse(adu, 2);
  else {
    uint8_t byteCount = div8RndUp(_quantity);
    adu.data[0] = byteCount;
    for (uint16_t i = 0; i < (byteCount * 8); i++) {
      uint8_t byteIndex = 1 + (i / 8);
      uint8_t bitIndex = i % 8;
      if (i < _quantity) bitWrite(adu.data[byteIndex], bitIndex, buf[_address + i]);
      else bitClear(adu.data[byteIndex], bitIndex);
    }
    adu.setDataLen(1 + byteCount);
  }
}

void ModbusSlaveLogic::_processReadValues(ModbusADU& adu, uint16_t buf[], uint16_t bufSize) {
  if (adu.getUnitId() == 0) return;
  _address = adu.getDataRegister(0);
  _quantity = adu.getDataRegister(2);
  if (!buf || bufSize == 0) _prepareExceptionResponse(adu, 1);
  else if (_quantity == 0 || _quantity > 125) _prepareExceptionResponse(adu, 3);
  else if (_quantity > bufSize || _address > (bufSize - _quantity)) _prepareExceptionResponse(adu, 2);
  else {
    uint8_t byteCount = _quantity * 2;
    adu.data[0] = byteCount;
    for (uint16_t i = 0; i < _quantity; i++) {
      adu.setDataRegister(1 + (i * 2), buf[_address + i]);
    }
    adu.setDataLen(1 + byteCount);
  }
}



void ModbusSlaveLogic::_prepareExceptionResponse(ModbusADU& adu, uint8_t exceptionCode) {
  _exceptionResponse = exceptionCode;
  adu.prepareExceptionResponse(_exceptionResponse);
}
