#include "Arduino.h"

using byte = uint8_t;

constexpr size_t c_maxPaylod = 1024;
constexpr byte c_preambleA = 0xb5;
constexpr byte c_preambleB = 0x62;

struct ubxMsg {
    byte cls;
    byte id;
    uint16_t len;
    byte checksumA;
    byte checksumB;
    byte payload[c_maxPaylod];
};

class UbloxReader
{
private:
    Stream& m_source;
    ubxMsg m_buffer;
        
    enum class ReaderState {
       Preamble,
       Class,
       ID,
       Length,
       Payload,
       Checksum,
    };

public:
    UbloxReader(Stream& source) : m_source(source) {};
    ubxMsg& read();
};

ubxMsg& UbloxReader::read()
{
    bool done = false;
    auto state = ReaderState::Preamble;

    while(!done)
    {
        
    }
}