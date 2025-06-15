package inet.routing.fsr;
import inet.applications.contract.IApp;

simple Fsr like IApp
{
 parameters:
        @class("fsr::Fsr");
        @display("i=block/routing");
        string routingTableModule = default("^.ipv4.routingTable");
        string interfaceTableModule = default("^.interfaceTable");
        string networkProtocolModule = default("^.ipv4.ip");
        string interface = default("wlan0");
        double lspUpdateInterval @unit(s) = default(0.5s);
        double lspLifeTimeInterval @unit(s) = default(0.5s);
        double maxWaitTimeForLspAnnouncement @unit(s) = default(4s);
        double helloBroadcastInterval @unit(s) = default(0.5s); 
        double maxJitter @unit(s) = default(50ms);
    gates:
        input socketIn;
        output socketOut;
}
