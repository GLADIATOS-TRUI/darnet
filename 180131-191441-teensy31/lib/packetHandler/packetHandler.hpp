#ifndef _PACKET_HANDLER_H_
#define _PACKET_HANDLER_H_

#ifndef _STDINT_H_
#define _STDINT_H_

#include <stdint.h>

#endif

#ifndef _VECTOR_
#define _VECTOR_

#include <vector>

#endif

#define bytes uint8_t

///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

class DynamixelPacket{
    bytes *biPacket = 0;
    uint8_t arrsize;
    void checksum(uint8_t *data_blk_ptr, uint8_t data_blk_size, uint8_t idx);
    int txpacket(uint8_t *txpacket);

    public:
        DynamixelPacket( bytes aID, bytes aInstruction, std::vector< bytes > adata );
        ~DynamixelPacket();
        void transaction();
};

#undef bytes
#endif