#include "inet/routing/fsr/CustomFsrRouting.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/ipv4/Ipv4RoutingTable.h"
#include "inet/networklayer/ipv4/Ipv4InterfaceData.h"
#include "inet/common/ModuleAccess.h"

namespace inet {
namespace customfsr {

Define_Module(CustomFsrRouting);

CustomFsrRouting::CustomFsrRouting() {}

CustomFsrRouting::~CustomFsrRouting() {
    cancelAndDelete(helloMsg);
    cancelAndDelete(topoUpdateMsg);
    cancelAndDelete(routeCleanupMsg);

    for (auto& entry : neighborTimers) {
        cancelAndDelete(entry.second);
    }
}

void CustomFsrRouting::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        interfaceTable = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        routingTable = getModuleFromPar<IRoutingTable>(par("routingTableModule"), this);
        udpCommSocket.setOutputGate(gate("udpOut"));
        udpCommSocket.setCallback(this);

        helloMsg = new cMessage("HelloTimer");
        topoUpdateMsg = new cMessage("UpdateTimer");
        routeCleanupMsg = new cMessage("CleanupTimer");

        helloDelay = par("helloInterval");
        updateDelay = par("updateInterval");
        timeJitter = par("jitter");

        configureLocalNode();
        initiateTimers();
    }
}

void CustomFsrRouting::configureLocalNode() {
    self = L3AddressResolver().resolve(getParentModule());
    udpCommSocket.bind(5000); // FSR port, can be parameterized
}

void CustomFsrRouting::initiateTimers() {
    scheduleAt(simTime() + uniform(0, timeJitter), helloMsg);
    scheduleAt(simTime() + updateDelay + uniform(0, timeJitter), topoUpdateMsg);
}

void CustomFsrRouting::handleMessageWhenUp(cMessage *msg) {
    if (msg == helloMsg) {
        sendHelloMessage();
        scheduleAt(simTime() + helloDelay + uniform(0, timeJitter), helloMsg);
    }
    else if (msg == topoUpdateMsg) {
        sendPeriodicUpdate();
        scheduleAt(simTime() + updateDelay + uniform(0, timeJitter), topoUpdateMsg);
    }
    else {
        EV_WARN << "Unknown internal message received: " << msg->getName() << endl;
    }
}

void CustomFsrRouting::handleStartOperation(LifecycleOperation *operation) {
    // Additional startup procedures if needed
}

void CustomFsrRouting::socketDataArrived(UdpSocket *sock, Packet *pkt) {
    auto fsrChunk = pkt->peekAtFront<FsrPacket>();
    handleIncomingPacket(fsrChunk, pkt->getTag<L3AddressInd>()->getSrcAddress());
    delete pkt;
}

void CustomFsrRouting::sendHelloMessage() {
    LinkStateEntry entry;
    entry.destAddress = self;
    entry.sequenceNumber = ++linkStateDatabase[self].sequenceId;
    entry.neighborsStartIndex = 0;
    entry.neighborsSize = 0;

    std::vector<LinkStateEntry> entries = { entry };
    auto packet = createUpdatePacket(entries);

    Packet *helloPkt = new Packet("FSR-Hello");
    helloPkt->insertAtBack(packet);
    udpCommSocket.sendTo(helloPkt, Ipv4Address::ALLONES_ADDRESS, 5000);
}

void CustomFsrRouting::sendPeriodicUpdate() {
    std::vector<LinkStateEntry> entries;
    for (const auto& pair : linkStateDatabase) {
        LinkStateEntry entry;
        entry.destAddress = pair.first;
        entry.sequenceNumber = pair.second.sequenceId;
        entry.neighborsStartIndex = 0;
        entry.neighborsSize = pair.second.adjacentNodes.size();
        entries.push_back(entry);
    }

    auto updatePacket = createUpdatePacket(entries);
    Packet *fsrUpdate = new Packet("FSR-Update");
    fsrUpdate->insertAtBack(updatePacket);
    udpCommSocket.sendTo(fsrUpdate, Ipv4Address::ALLONES_ADDRESS, 5000);
}

void CustomFsrRouting::handleIncomingPacket(const Ptr<const FsrPacket>& fsrPkt, const L3Address& src) {
    Ipv4Address source = src.toIpv4();

    for (const auto& entry : fsrPkt->getLinks()) {
        auto& node = linkStateDatabase[entry.destAddress];

        if (entry.sequenceNumber > node.sequenceId) {
            node.sequenceId = entry.sequenceNumber;
            node.timestamp = simTime().raw();
        }
    }

    floodTopologyData();
    runDijkstra();
}

void CustomFsrRouting::floodTopologyData() {
    // Currently handled by sendPeriodicUpdate()
}

void CustomFsrRouting::runDijkstra() {
    std::map<Ipv4Address, Ipv4Address> previousHop;

    distances.clear();
    distances[self] = 0;

    std::set<Ipv4Address> visited;
    std::set<Ipv4Address> unvisited = { self };

    while (!unvisited.empty()) {
        Ipv4Address current;
        uint32_t minDist = UINT32_MAX;
        for (const auto& node : unvisited) {
            if (distances[node] < minDist) {
                current = node;
                minDist = distances[node];
            }
        }

        unvisited.erase(current);
        visited.insert(current);

        for (const auto& neighbor : linkStateDatabase[current].adjacentNodes) {
            uint32_t newDist = distances[current] + 1;
            if (!distances.count(neighbor) || newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previousHop[neighbor] = current;
                unvisited.insert(neighbor);
            }
        }
    }

    updateRoutingTable(previousHop);
}

void CustomFsrRouting::updateRoutingTable(const std::map<Ipv4Address, Ipv4Address>& pathMap) {
    routingTable->purge();
    for (const auto& dest : distances) {
        if (dest.first == self) continue;

        Ipv4Address nextHop = dest.first;
        while (pathMap.at(nextHop) != self) {
            nextHop = pathMap.at(nextHop);
        }

        insertRoute(dest.first, nextHop, dest.second);
    }
}

void CustomFsrRouting::insertRoute(const Ipv4Address& dest, const Ipv4Address& nextHop, uint32_t hopCount) {
    Ipv4Route *route = new Ipv4Route();
    route->setDestination(dest);
    route->setNetmask(Ipv4Address::ALLONES_ADDRESS);
    route->setGateway(nextHop);
    route->setInterface(interfaceTable->getFirstLoopbackInterface()); // You may want to choose a better interface
    route->setSourceType(IRoute::FSR);
    routingTable->addRoute(route);
}

Ptr<FsrPacket> CustomFsrRouting::createUpdatePacket(const std::vector<LinkStateEntry>& entries) {
    auto fsrPkt = makeShared<FsrPacket>();
    fsrPkt->setLinks(entries);
    fsrPkt->setChunkLength(B(32 + entries.size() * 16)); // arbitrary size estimate
    return fsrPkt;
}

void CustomFsrRouting::clearObsoleteRoutes() {
    // Optional: implement timeout-based route purging
}

void CustomFsrRouting::logTopology() {
    EV_INFO << "FSR Topology Table:\n";
    for (const auto& pair : linkStateDatabase) {
        EV_INFO << "Node: " << pair.first << " Seq: " << pair.second.sequenceId << "\n";
    }
}

void CustomFsrRouting::finish() {
    logTopology();
}

} // namespace customfsr
} // namespace inet
