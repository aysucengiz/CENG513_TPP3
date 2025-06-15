#ifndef INET_ROUTING_FSR_CUSTOMFSRROUTING_H_
#define INET_ROUTING_FSR_CUSTOMFSRROUTING_H_

#include "inet/common/packet/Packet.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/contract/IRoutingTable.h"
#include "inet/networklayer/contract/ipv4/Ipv4Address.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "inet/routing/fsr/FsrPacket_m.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"

#include <map>
#include <set>

namespace inet {
namespace customfsr {

class INET_API CustomFsrRouting : public RoutingProtocolBase, public UdpSocket::ICallback
{
  private:
    struct NodeInfo {
        std::set<Ipv4Address> adjacentNodes;
        uint32_t sequenceId = 0;
        uint32_t timestamp = 0;
    };

    UdpSocket udpCommSocket;
    Ipv4Address localAddress;

    ModuleRefByPar<IInterfaceTable> interfaceTable;
    ModuleRefByPar<IRoutingTable> routingTable;

    cMessage *helloMsg = nullptr;
    cMessage *topoUpdateMsg = nullptr;
    cMessage *routeCleanupMsg = nullptr;

    double helloDelay = 1;
    double updateDelay = 3;
    double timeJitter = 0.1;

    std::map<Ipv4Address, NodeInfo> linkStateDatabase;
    std::map<Ipv4Address, uint32_t> costMap;
    std::map<Ipv4Address, cMessage *> neighborTimers;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;

    virtual void handleStartOperation(LifecycleOperation *operation) override;

    virtual void socketDataArrived(UdpSocket *sock, Packet *pkt) override;

  private:
    void configureLocalNode();
    void initiateTimers();

    void handleIncomingPacket(const Ptr<const FsrPacket>& fsrPkt, const L3Address& src);
    void floodTopologyData();
    void runDijkstra();
    void updateRoutingTable(const std::map<Ipv4Address, Ipv4Address>& pathMap);

    void sendHelloMessage();
    void sendPeriodicUpdate();

    Ptr<FsrPacket> createUpdatePacket(const std::vector<LinkStateEntry>& entries);
    void insertRoute(const Ipv4Address& destination, const Ipv4Address& forwardHop, uint32_t metric);
    void clearObsoleteRoutes();
    void logTopology();

  public:
    CustomFsrRouting();
    virtual ~CustomFsrRouting() override;
};

} // namespace customfsr
} // namespace inet

#endif // INET_ROUTING_FSR_CUSTOMFSRROUTING_H_
