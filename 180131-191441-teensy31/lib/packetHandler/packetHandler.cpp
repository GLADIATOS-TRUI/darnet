#include <packetHandler.hpp>
#include <Arduino.h>
//#include <stdlib.h>

#define __DYN_SERIAL__ Serial2

DynamixelPacket::DynamixelPacket( uint8_t aID, uint8_t aInstruction, std::vector< uint8_t > adata )
{
  uint8_t pos;
  uint8_t arr_size = adata.size()+6;
  this -> biPacket = (uint8_t *)malloc(sizeof(uint8_t) * (arr_size));
  this -> biPacket[PKT_HEADER0] = 0xFF;
  this -> biPacket[PKT_HEADER1] = 0xFF;
  this -> biPacket[PKT_ID] = aID;
  this -> biPacket[PKT_LENGTH] = arr_size - 4;
  this -> biPacket[PKT_INSTRUCTION] = aInstruction;
  for (int i = 0; i<(arr_size-6); i++){
    this -> biPacket[PKT_PARAMETER0+i] = adata[i];
  } 
  pos = arr_size - 1;
  checksum(this -> biPacket,arr_size,pos);
  this -> arrsize =  arr_size;
}

// calculate checksum
void DynamixelPacket::checksum(uint8_t *data_blk_ptr, uint8_t data_blk_size, uint8_t idx){
  uint8_t checksum = 0;
  for (uint16_t i = 2; i < data_blk_size - 1; i++)   // except header, checksum
    checksum += data_blk_ptr[i];
  checksum = ~checksum;
  data_blk_ptr[idx] = checksum;
}

DynamixelPacket::~DynamixelPacket(){
  free(this -> biPacket);
}

void DynamixelPacket::transaction(){
  __DYN_SERIAL__.flush();
  int length_written;
  length_written = __DYN_SERIAL__.write(this -> biPacket,this -> arrsize);
  __DYN_SERIAL__.flush();
  if (length_written != (this -> arrsize))
    Serial.println("ERROR");
  else
    Serial.println("SUCCEED");
  for (int i=0;i<this -> arrsize;i++)
  Serial.printf("%x ",this->biPacket[i]);
  
}
