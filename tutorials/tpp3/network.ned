//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


package inet.tutorials.tpp3;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.contract.INetworkNode;
import inet.physicallayer.wireless.common.contract.packetlevel.IRadioMedium;
import inet.visualizer.contract.IIntegratedVisualizer;
import inet.node.fsr.FsrRouter;
import inet.environment.common.PhysicalEnvironment;

network FSRtest
{
    parameters:
        int numHosts = default(28);
        @display("bgb=700,700;bgg=100,1,grey95");
        @figure[title](type=label; pos=0,-1; anchor=sw; color=darkblue);

        @figure[rcvdPkText](type=indicatorText; pos=380,20; anchor=w; font=,18; textFormat="packets received: %g"; initialValue=0);
        @statistic[packetReceived](source=hostB.app[0].packetReceived; record=figure(count); targetFigure=rcvdPkText);

    submodules:
        visualizer: <default(firstAvailableOrEmpty("IntegratedCanvasVisualizer"))> like IIntegratedVisualizer if typename != "" {
            @display("p=580,125");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=580,200");
        }
        radioMedium: <default("UnitDiskRadioMedium")> like IRadioMedium {
            @display("p=580,275");
        }
        physicalEnvironment: PhysicalEnvironment {
            @display("p=580,425");
        }
        hostA: <default("WirelessHost")> like INetworkNode {
            @display("p=50,325");
        }
        host1: <default("WirelessHost")> like INetworkNode {
            @display("p=184,324");
        }
//        host2: <default("WirelessHost")> like INetworkNode {
//            @display("p=250,325");
//        }
//                host3: <default("WirelessHost")> like INetworkNode {
//            @display("p=350,325");
//        }
//                host4: <default("WirelessHost")> like INetworkNode {
//            @display("p=450,325");
//        }
//                host5: <default("WirelessHost")> like INetworkNode {
//            @display("p=550,325");
//        }

        hostB: <default("WirelessHost")> like INetworkNode {
            @display("p=296,324");
        }
        host3: <default("WirelessHost")> like INetworkNode {
            @display("p=175,474");
        }
    //		hostR1: <default("WirelessHost")> like INetworkNode {
    //            @display("p=250,300");
    //        }
    //        hostR2: <default("WirelessHost")> like INetworkNode {
    //            @display("p=150,450");
    //        }
    //        hostR3: <default("WirelessHost")> like INetworkNode {
    //            @display("p=350,450");
    //        }
    		host[numHosts]: <default("WirelessHost")> like INetworkNode {
    		   
            }
    connections allowunconnected:
}
