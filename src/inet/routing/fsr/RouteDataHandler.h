#ifndef INET_ROUTING_FSR_ROUTEDATAHANDLER_H
#define INET_ROUTING_FSR_ROUTEDATAHANDLER_H

#include "inet/common/packet/serializer/FieldsChunkSerializer.h"

namespace inet {
namespace fsr {

class INET_API RouteDataHandler : public FieldsChunkSerializer
{
  protected:
    virtual void serialize(MemoryOutputStream& stream, const Ptr<const Chunk>& chunk) const override;
    virtual const Ptr<Chunk> deserialize(MemoryInputStream& stream) const override;

  public:
    RouteDataHandler() : FieldsChunkSerializer() {}
};

} // namespace fsr
} // namespace inet

#endif // INET_ROUTING_FSR_ROUTEDATAHANDLER_H
