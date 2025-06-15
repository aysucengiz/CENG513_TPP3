#include "inet/routing/fsr/RouteDataHandler.h"
#include "inet/routing/fsr/FsrPacket_m.h"

namespace inet {
namespace fsr {

Register_Serializer(FsrPacket, RouteDataHandler);

void RouteDataHandler::serialize(MemoryOutputStream& stream, const Ptr<const Chunk>& chunk) const
{
    const auto& pkt = staticPtrCast<const FsrPacket>(chunk);

    stream.writeUint32(pkt->getLinksArraySize());

    for (const auto& entry : pkt->getLinks()) {
        stream.writeIpv4Address(entry.destAddress);
        stream.writeUint32(entry.sequenceNumber);
        stream.writeUint32(entry.neighborsArraySize());

        for (const auto& neighbor : entry.neighbors) {
            stream.writeIpv4Address(neighbor);
        }
    }
}

const Ptr<Chunk> RouteDataHandler::deserialize(MemoryInputStream& stream) const
{
    auto pkt = makeShared<FsrPacket>();

    uint32_t linkCount = stream.readUint32();
    std::vector<LinkStateEntry> links;
    links.reserve(linkCount);

    for (uint32_t i = 0; i < linkCount; ++i) {
        LinkStateEntry entry;
        entry.destAddress = stream.readIpv4Address();
        entry.sequenceNumber = stream.readUint32();

        uint32_t neighborCount = stream.readUint32();
        std::vector<Ipv4Address> neighbors;
        neighbors.reserve(neighborCount);

        for (uint32_t j = 0; j < neighborCount; ++j) {
            neighbors.push_back(stream.readIpv4Address());
        }

        entry.neighbors = neighbors;
        links.push_back(entry);
    }

    pkt->setLinks(links);
    return pkt;
}

} // namespace fsr
} // namespace inet
