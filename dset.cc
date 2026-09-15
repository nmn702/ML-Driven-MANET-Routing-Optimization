#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/wifi-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/flow-monitor-module.h"
#include <iostream>

using namespace ns3;

void ConfigureAodv(AodvHelper& aodv, uint32_t ttl, uint32_t retryAttempts, const std::string& protocol) {
    aodv.Set("NetDiameter", UintegerValue(35)); 
    aodv.Set("TtlStart", UintegerValue(1));
    aodv.Set("TtlThreshold", UintegerValue(ttl)); 
    aodv.Set("RreqRetries", UintegerValue(retryAttempts));
    
    if (protocol == "AODV-AERS1") aodv.Set("TtlIncrement", UintegerValue(2));
    else if (protocol == "AODV-AERS2") aodv.Set("TtlIncrement", UintegerValue(1));
    else aodv.Set("TtlIncrement", UintegerValue(2)); 
}

int main(int argc, char *argv[]) {
    uint32_t nNodes = 60;
    double terrainSize = 1200.0;
    double nodeSpeed = 25.0;
    std::string trafficRate = "2048bps";
    uint32_t packetSize = 512;
    uint32_t ttl = 3;
    double waitTime = 1.0;
    uint32_t retryAttempts = 3;
    double routeExpiry = 3.0;
    uint32_t bufferSize = 100;
    double threshMin = 5.0;
    double threshMax = 15.0;
    double dropProb = 0.05;
    std::string routingProtocol = "AODV";

    CommandLine cmd(__FILE__);
    cmd.AddValue("nNodes", "Number of nodes", nNodes);
    cmd.AddValue("terrainSize", "Terrain size", terrainSize);
    cmd.AddValue("nodeSpeed", "Node speed", nodeSpeed);
    cmd.AddValue("trafficRate", "Traffic rate", trafficRate);
    cmd.AddValue("packetSize", "Packet size", packetSize);
    cmd.AddValue("ttl", "TTL", ttl);
    cmd.AddValue("waitTime", "Wait time", waitTime);
    cmd.AddValue("retryAttempts", "Retry attempts", retryAttempts);
    cmd.AddValue("routeExpiry", "Route expiry", routeExpiry);
    cmd.AddValue("bufferSize", "Buffer size", bufferSize);
    cmd.AddValue("threshMin", "Min threshold", threshMin);
    cmd.AddValue("threshMax", "Max threshold", threshMax);
    cmd.AddValue("dropProb", "Drop prob", dropProb);
    cmd.AddValue("routingProtocol", "Protocol", routingProtocol);
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(nNodes);
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(250.0));
    wifiPhy.SetChannel(wifiChannel.Create());
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", 
        "DataMode", StringValue("DsssRate11Mbps"), 
        "ControlMode", StringValue("DsssRate11Mbps"));
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(terrainSize) + "]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(terrainSize) + "]"));
    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

    MobilityHelper mobility;
    mobility.SetPositionAllocator(taPositionAlloc);
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(nodeSpeed) + "]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
        "PositionAllocator", PointerValue(taPositionAlloc));
    mobility.Install(nodes);

    InternetStackHelper internet;
    if (routingProtocol == "OLSR") {
        OlsrHelper olsr;
        internet.SetRoutingHelper(olsr);
    } else if (routingProtocol == "DSDV") {
        DsdvHelper dsdv;
        internet.SetRoutingHelper(dsdv);
    } else {
        AodvHelper aodv;
        ConfigureAodv(aodv, ttl, retryAttempts, routingProtocol);
        internet.SetRoutingHelper(aodv);
    }
    internet.Install(nodes);

    if (routingProtocol == "AODV-AERS3") {
        TrafficControlHelper tc;
        tc.SetRootQueueDisc("ns3::RedQueueDisc",
            "MaxSize", StringValue(std::to_string(bufferSize) + "p"),
            "MinTh", DoubleValue(threshMin),
            "MaxTh", DoubleValue(threshMax),
            "LInterm", DoubleValue(1.0 / dropProb));
        tc.Install(devices);
    }

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);
    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(0));
    serverApps.Start(Seconds(5.0));
    serverApps.Stop(Seconds(35.0));

    UdpEchoClientHelper echoClient(interfaces.GetAddress(0), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.05)));
    echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
    ApplicationContainer clientApps = echoClient.Install(nodes.Get(nNodes - 1));
    clientApps.Start(Seconds(6.0));
    clientApps.Stop(Seconds(35.0));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(35.0));
    Simulator::Run();

    monitor->CheckForLostPackets();
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    double rxBytes = 0, rxPackets = 0, txPackets = 0, delaySum = 0;
    for (auto const& stat : stats) {
        txPackets += stat.second.txPackets;
        rxPackets += stat.second.rxPackets;
        rxBytes += stat.second.rxBytes;
        if (stat.second.rxPackets > 0) delaySum += stat.second.delaySum.GetSeconds();
    }

    if (rxPackets > 0) {
        std::cout << "Throughput: " << (rxBytes * 8.0) / 29.0 / 1024.0 << " kbps" << std::endl;
        std::cout << "PDR: " << (rxPackets / txPackets) * 100.0 << " %" << std::endl;
        std::cout << "E2E Delay: " << (delaySum / rxPackets) * 1000.0 << " ms" << std::endl;
    } else {
        std::cout << "Throughput: 0.0 kbps\nPDR: 0.0 %\nE2E Delay: 0.0 ms\n";
    }

    Simulator::Destroy();
    return 0;
}