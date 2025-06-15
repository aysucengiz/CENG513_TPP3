//
// FsrPacket.msg
//
import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.networklayer.contract.ipv4.Ipv4Address;

namespace inet;
struct LinkStateEntry
{
    Ipv4Address destAddress;       
    uint32 sequenceNumber;        
    Ipv4Address neighbors[];     
}

class FsrPacket extends FieldsChunk
{
    LinkStateEntry links[];     
}

