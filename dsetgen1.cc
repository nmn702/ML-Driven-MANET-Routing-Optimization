#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>
using namespace ns3;
// GLOBAL PARAMETERS
static constexpr uint32_t NUM_SCENARIOS = 1000;
static constexpr double SIM_TIME = 25.0;
static constexpr double APP_START = 8.0;
static constexpr double APP_STOP  = 23.0;
static constexpr double RADIO_RANGE = 250.0;
static constexpr uint16_t APP_PORT = 9000;
static constexpr uint32_t RNG_SEED = 91731;
static constexpr double PI = 3.14159265358979323846;
// SCENARIO
struct Scenario
{
    uint32_t id;
    std::string topology;
    uint32_t nodes;
    double terrain;
    double nodeDensity;
    double speed;
    uint64_t trafficRate;
    uint32_t packetSize;
    uint32_t ttl;
    double waitTime;
    uint32_t retryAttempts;
    double routeExpiry;
    uint32_t bufferSize;
    double threshMin;
    double threshMax;
    double dropProbability;
    uint32_t source;
    uint32_t destination;
    int initialHops;
    std::vector<Vector> positions;
};
// PROTOCOL RESULT
struct ProtocolResult
{
    double throughputMbps = 0.0;
    double pdr = 0.0;
    double delayMs = -1.0;
    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
};
struct ProtocolRun
{
    std::string name;
    ProtocolResult result;
};
// RANDOM HELPERS
static double
RandDouble(std::mt19937& rng, double low, double high)
{
    std::uniform_real_distribution<double> dist( low, high);
    return dist(rng);
}
static uint32_t
RandUInt(std::mt19937& rng, uint32_t low, uint32_t high)
{
    std::uniform_int_distribution<uint32_t> dist( low, high);
    return dist(rng);
}
static uint64_t
RandUInt64(std::mt19937& rng, uint64_t low, uint64_t high)
{
    std::uniform_int_distribution<uint64_t> dist( low, high);
    return dist(rng);
}
// GEOMETRY
static double
Distance(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt( dx * dx + dy * dy);
}
// CONNECTIVITY
static std::vector<int>
GetComponents(const std::vector<Vector>& positions)
{
    const uint32_t n = static_cast<uint32_t>(positions.size());
    std::vector<int> component( n, -1);
    int componentId = 0;
    for (uint32_t start = 0; start < n; ++start)
    {
        if (component[start] != -1)
        {
            continue;
        }
    std::vector<uint32_t> queue;
    queue.push_back(start);
    component[start] = componentId;
    for (size_t q = 0; q < queue.size(); ++q)
    {
        uint32_t u = queue[q];
        for (uint32_t v = 0; v < n; ++v)
        {
            if (component[v] != -1)
            {
                continue;
            }
        if (Distance( positions[u], positions[v]) <= RADIO_RANGE)
        {
            component[v] = componentId;
            queue.push_back(v);
        }
}
}
++componentId;
}
return component;
}
// KEEP GENERATED TOPOLOGY CONNECTED
static void
EnsureConnected(std::vector<Vector>& positions, double terrain)
{
    if (positions.size() <= 1)
    {
        return;
    }
const uint32_t n = static_cast<uint32_t>(
positions.size());
for (uint32_t iteration = 0; iteration < n * 3; ++iteration)
{
    std::vector<int> components = GetComponents( positions);
    int maxComponent = 0;
    for (int c : components)
    {
        maxComponent = std::max( maxComponent, c);
    }
if (maxComponent == 0)
{
    return;
}
double bestDistance = std::numeric_limits<double>::max();
uint32_t bestA = 0;
uint32_t bestB = 0;
for (uint32_t i = 0; i < n; ++i)
{
    for (uint32_t j = i + 1; j < n; ++j)
    {
        if (components[i] == components[j])
        {
            continue;
        }
    double d = Distance( positions[i], positions[j]);
    if (d < bestDistance)
    {
        bestDistance = d;
        bestA = i;
        bestB = j;
    }
}
}
if (bestDistance == std::numeric_limits<double>::max())
{
    return;
}
Vector a = positions[bestA];
Vector b = positions[bestB];
double dx =b.x - a.x;
double dy =b.y - a.y;
double length = std::sqrt( dx * dx + dy * dy);
if (length < 1e-9)
{
    positions[bestB].x = std::min( terrain - 1.0, positions[bestB].x + RADIO_RANGE * 0.5);
    continue;
}
// Put the disconnected component
// within radio range of this component.
double targetDistance =RADIO_RANGE * 0.75;
Vector newPosition;
newPosition.x =a.x +
(dx / length) *
targetDistance;
newPosition.y =a.y +
(dy / length) *
targetDistance;
newPosition.z = 0.0;
newPosition.x = std::max( 1.0, std::min( terrain - 1.0, newPosition.x));
newPosition.y = std::max( 1.0, std::min( terrain - 1.0, newPosition.y));
positions[bestB] = newPosition;
}
}
// INITIAL HOP COUNT
static int
GetInitialHops( const std::vector<Vector>& positions, uint32_t source, uint32_t destination)
{
    const uint32_t n = static_cast<uint32_t>(
    positions.size());
    std::vector<int> distance( n, -1);
    std::vector<uint32_t> queue;
    distance[source] = 0;
    queue.push_back(source);
    for (size_t q = 0; q < queue.size(); ++q)
    {
        uint32_t u = queue[q];
        if (u == destination)
        {
            return distance[u];
        }
    for (uint32_t v = 0; v < n; ++v)
    {
        if (distance[v] != -1)
        {
            continue;
        }
    if (Distance( positions[u], positions[v]) <= RADIO_RANGE)
    {
        distance[v] =distance[u] + 1;
        queue.push_back(v);
    }
}
}
return -1;
}
// SOURCE / DESTINATION
static std::pair<uint32_t, uint32_t>
ChooseFarthestPair( const std::vector<Vector>& positions)
{
    double maximumDistance = -1.0;
    uint32_t source = 0;
    uint32_t destination = positions.size() > 1 ? 1 : 0;
    for (uint32_t i = 0; i < positions.size(); ++i)
    {
        for (uint32_t j = i + 1; j < positions.size(); ++j)
        {
            double d = Distance( positions[i], positions[j]);
            if (d > maximumDistance)
            {
                maximumDistance = d;
                source = i;
                destination = j;
            }
    }
}
return {
    source,
    destination
};
}
// TOPOLOGY HELPERS
static Vector
ClampPosition(Vector p, double terrain)
{
    p.x = std::max( 1.0, std::min( terrain - 1.0, p.x));
    p.y = std::max( 1.0, std::min( terrain - 1.0, p.y));
    p.z = 0.0;
    return p;
}
// RANDOM TOPOLOGY
static void
GenerateRandomTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        positions[i] =Vector( RandDouble( rng, 10.0, terrain - 10.0), RandDouble( rng, 10.0, terrain - 10.0), 0.0);
    }
}
// GRID
static void
GenerateGridTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    uint32_t columns = static_cast<uint32_t>(
    std::ceil(
    std::sqrt(
    static_cast<double>(n))));
    uint32_t rows = static_cast<uint32_t>(
    std::ceil(
    static_cast<double>(n) /
    static_cast<double>(columns)));
    double margin = terrain * 0.08;
    double usable = terrain -
    2.0 * margin;
    double dx = columns > 1
    ? usable /
    static_cast<double>(
    columns - 1)
    : 0.0;
    double dy = rows > 1
    ? usable /
    static_cast<double>(
    rows - 1)
    : 0.0;
    for (uint32_t i = 0; i < n; ++i)
    {
        uint32_t row = i / columns;
        uint32_t column = i % columns;
        double jitterX = RandDouble( rng, -0.12 * dx, 0.12 * dx);
        double jitterY = RandDouble( rng, -0.12 * dy, 0.12 * dy);
        positions[i] =ClampPosition( Vector( margin + column * dx + jitterX, margin + row * dy + jitterY, 0.0), terrain);
    }
}
// LINE
static void
GenerateLineTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    double start = terrain * 0.06;
    double end = terrain * 0.94;
    double step = n > 1
    ? (end - start) /
    static_cast<double>(
    n - 1)
    : 0.0;
    double center = terrain * 0.5;
    for (uint32_t i = 0; i < n; ++i)
    {
        double x = start +
        i * step;
        double y = center +
        RandDouble( rng, -terrain * 0.04, terrain * 0.04);
        positions[i] =ClampPosition( Vector( x, y, 0.0), terrain);
    }
}
// CORRIDOR
static void
GenerateCorridorTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    double center = terrain * 0.5;
    for (uint32_t i = 0; i < n; ++i)
    {
        double x = RandDouble( rng, terrain * 0.05, terrain * 0.95);
        double y = center +
        RandDouble( rng, -terrain * 0.10, terrain * 0.10);
        positions[i] =ClampPosition( Vector( x, y, 0.0), terrain);
    }
}
// CLUSTER POINT
static Vector
RandomClusterPoint( const Vector& center, double radius, double terrain, std::mt19937& rng)
{
    double angle = RandDouble( rng, 0.0, 2.0 * PI);
    double radiusValue = radius *
    std::sqrt( RandDouble( rng, 0.0, 1.0));
    Vector p;
    p.x = center.x +
    radiusValue *
    std::cos(angle);
    p.y = center.y +
    radiusValue *
    std::sin(angle);
    p.z = 0.0;
    return ClampPosition( p, terrain);
}
// TWO CLUSTERS
static void
GenerateTwoClusterTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    Vector left( terrain * 0.25, terrain * 0.50, 0.0);
    Vector right( terrain * 0.75, terrain * 0.50, 0.0);
    uint32_t bridgeNodes = std::max( uint32_t(4), n / 10);
    for (uint32_t i = 0; i < n; ++i)
    {
        if (i < bridgeNodes)
        {
            double t = static_cast<double>(i) /
            static_cast<double>(
            bridgeNodes - 1);
            double x = left.x * (1.0 - t) +
            right.x * t;
            double y = terrain * 0.50 +
            RandDouble( rng, -terrain * 0.05, terrain * 0.05);
            positions[i] =ClampPosition( Vector( x, y, 0.0), terrain);
        }
    else if (i % 2 == 0)
    {
        positions[i] = RandomClusterPoint( left, terrain * 0.20, terrain, rng);
    }
else
{
    positions[i] = RandomClusterPoint( right, terrain * 0.20, terrain, rng);
}
}
}
// HOTSPOT
static void
GenerateHotspotTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    Vector center( terrain * 0.50, terrain * 0.50, 0.0);
    uint32_t hotspotNodes = static_cast<uint32_t>(
    std::round(
    0.70 *
    static_cast<double>(n)));
    for (uint32_t i = 0; i < n; ++i)
    {
        if (i < hotspotNodes)
        {
            positions[i] = RandomClusterPoint( center, terrain * 0.20, terrain, rng);
        }
    else
    {
        positions[i] =Vector( RandDouble( rng, 10.0, terrain - 10.0), RandDouble( rng, 10.0, terrain - 10.0), 0.0);
    }
}
}
// RING
static void
GenerateRingTopology( std::vector<Vector>& positions, uint32_t n, double terrain, std::mt19937& rng)
{
    Vector center( terrain * 0.50, terrain * 0.50, 0.0);
    double radius = terrain * 0.30;
    for (uint32_t i = 0; i < n; ++i)
    {
        double angle = 2.0 *
        PI *
        static_cast<double>(i) /
        static_cast<double>(n);
        angle += RandDouble( rng, -0.05, 0.05);
        double r = radius +
        RandDouble( rng, -terrain * 0.03, terrain * 0.03);
        positions[i] =ClampPosition( Vector( center.x + r * std::cos(angle), center.y + r * std::sin(angle), 0.0), terrain);
    }
}
// TOPOLOGY GENERATOR
static void
GenerateTopology( Scenario& s, uint32_t topologyType, std::mt19937& rng)
{
    s.positions.resize( s.nodes);
    switch (topologyType)
    {
    case 0:
        s.topology = "RANDOM";
        GenerateRandomTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    case 1:
        s.topology = "GRID";
        GenerateGridTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    case 2:
        s.topology = "LINE";
        GenerateLineTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    case 3:
        s.topology = "CORRIDOR";
        GenerateCorridorTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    case 4:
        s.topology = "TWO_CLUSTER";
        GenerateTwoClusterTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    case 5:
        s.topology = "HOTSPOT";
        GenerateHotspotTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    case 6:
        s.topology = "RING";
        GenerateRingTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    default:
        s.topology = "RANDOM";
        GenerateRandomTopology( s.positions, s.nodes, s.terrain, rng);
        break;
    }
// Guarantee a usable initial connected topology.
EnsureConnected( s.positions, s.terrain);
// Select the farthest pair.
// This creates meaningful multi-hop MANET traffic.
auto pair =ChooseFarthestPair( s.positions);
s.source = pair.first;
s.destination = pair.second;
s.initialHops = GetInitialHops( s.positions, s.source, s.destination);
}
// SCENARIO GENERATOR
static Scenario
GenerateScenario( uint32_t id, uint32_t topologyType, std::mt19937& rng)
{
    Scenario s;
    s.id = id;
    // Different scenario families deliberately sample very
    // different MANET regimes.
    uint32_t family = RandUInt( rng, 0, 7);
    switch (family)
    {
        // Dense / compact
    case 0:
        {
            s.nodes = RandUInt( rng, 50, 80);
            s.terrain = RandDouble( rng, 350.0, 700.0);
            s.speed = RandDouble( rng, 0.5, 6.0);
            s.trafficRate = RandUInt64( rng, 256000, 1800000);
            break;
        }
    // Sparse / large
case 1:
    {
        s.nodes = RandUInt( rng, 20, 45);
        s.terrain = RandDouble( rng, 800.0, 1800.0);
        s.speed = RandDouble( rng, 1.0, 14.0);
        s.trafficRate = RandUInt64( rng, 64000, 600000);
        break;
    }
// Medium
case 2:
    {
        s.nodes = RandUInt( rng, 30, 65);
        s.terrain = RandDouble( rng, 500.0, 1200.0);
        s.speed = RandDouble( rng, 0.2, 12.0);
        s.trafficRate = RandUInt64( rng, 128000, 1400000);
        break;
    }
// High mobility
case 3:
    {
        s.nodes = RandUInt( rng, 25, 70);
        s.terrain = RandDouble( rng, 500.0, 1500.0);
        s.speed = RandDouble( rng, 8.0, 18.0);
        s.trafficRate = RandUInt64( rng, 64000, 1000000);
        break;
    }
// Low mobility + congestion
case 4:
    {
        s.nodes = RandUInt( rng, 45, 80);
        s.terrain = RandDouble( rng, 400.0, 900.0);
        s.speed = RandDouble( rng, 0.1, 2.5);
        s.trafficRate = RandUInt64( rng, 800000, 2200000);
        break;
    }
// Extremely heterogeneous
case 5:
    {
        s.nodes = RandUInt( rng, 20, 80);
        s.terrain = RandDouble( rng, 400.0, 1800.0);
        s.speed = RandDouble( rng, 0.2, 16.0);
        s.trafficRate = RandUInt64( rng, 64000, 2200000);
        break;
    }
// Cluster-heavy
case 6:
    {
        s.nodes = RandUInt( rng, 35, 75);
        s.terrain = RandDouble( rng, 600.0, 1500.0);
        s.speed = RandDouble( rng, 0.5, 12.0);
        s.trafficRate = RandUInt64( rng, 128000, 1800000);
        break;
    }
// Long-range sparse MANET
default:
    {
        s.nodes = RandUInt( rng, 20, 55);
        s.terrain = RandDouble( rng, 900.0, 1900.0);
        s.speed = RandDouble( rng, 2.0, 15.0);
        s.trafficRate = RandUInt64( rng, 64000, 800000);
        break;
    }
}
// PACKET SIZE
uint32_t packetClass = RandUInt( rng, 0, 4);
switch (packetClass)
{
case 0:
    s.packetSize = RandUInt( rng, 128, 256);
    break;
case 1:
    s.packetSize = RandUInt( rng, 256, 512);
    break;
case 2:
    s.packetSize = RandUInt( rng, 512, 800);
    break;
case 3:
    s.packetSize = RandUInt( rng, 800, 1200);
    break;
default:
    s.packetSize = RandUInt( rng, 1200, 1400);
    break;
}
// QUEUE
s.bufferSize = RandUInt( rng, 25, 180);
// RED minimum threshold.
s.threshMin = std::max( 3.0, std::round( static_cast<double>( s.bufferSize) * RandDouble( rng, 0.12, 0.25)));
// RED maximum threshold.
s.threshMax = std::round( static_cast<double>( s.bufferSize) * RandDouble( rng, 0.45, 0.70));
if (s.threshMax <= s.threshMin)
{
    s.threshMax = s.threshMin + 4.0;
}
s.threshMax = std::min( s.threshMax, static_cast<double>( s.bufferSize - 1));
s.dropProbability = RandDouble( rng, 0.02, 0.20);
// TOPOLOGY
GenerateTopology( s, topologyType, rng);
// AERS / AODV PARAMETERS
uint32_t minimumTtl = static_cast<uint32_t>(
std::max(
3,
s.initialHops + 2));
s.ttl = minimumTtl +
RandUInt( rng, 0, 5);
s.ttl = std::max( uint32_t(5), std::min( uint32_t(25), s.ttl));
s.retryAttempts = RandUInt( rng, 1, 4);
s.waitTime = RandDouble( rng, 0.5, 3.5);
s.routeExpiry = RandDouble( rng, 1.0, 6.0);
// nodes / km^2
s.nodeDensity = static_cast<double>(
s.nodes) /
(s.terrain * s.terrain) *
1000000.0;
return s;
}
// AODV CONFIGURATION
static void
ConfigureAodv( AodvHelper& aodv, const Scenario& s, const std::string& protocol)
{
    // 1. Restore a safe network diameter so AODV timers do not overlap with MAC broadcasts
    aodv.Set("NetDiameter", UintegerValue(35));
    // 2. Apply the parametric TTL to the actual Expanding Ring Search threshold
    aodv.Set("TtlStart", UintegerValue(1));
    aodv.Set("TtlThreshold", UintegerValue(s.ttl));
    aodv.Set("RreqRetries", UintegerValue(s.retryAttempts));
    // 3. Define the customized AERS variants by tweaking the TTL increment steps
    if (protocol == "AODV-AERS1")
    {
        aodv.Set("TtlIncrement", UintegerValue(2));
    }
else if (protocol == "AODV-AERS2")
{
    aodv.Set("TtlIncrement", UintegerValue(1));
}
else if (protocol == "AODV-AERS3")
{
    aodv.Set("TtlIncrement", UintegerValue(2));
}
else
{
    aodv.Set("TtlIncrement", UintegerValue(2));
}
}
// WIFI
static NetDeviceContainer
InstallNetworkDevices( NodeContainer& nodes)
{
    WifiHelper wifi;
    wifi.SetStandard( WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager( "ns3::ConstantRateWifiManager", "DataMode", StringValue( "DsssRate11Mbps"), "ControlMode", StringValue( "DsssRate11Mbps"));
    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay( "ns3::ConstantSpeedPropagationDelayModel");
    // Explicit MANET communication range.
    channel.AddPropagationLoss( "ns3::RangePropagationLossModel", "MaxRange", DoubleValue( RADIO_RANGE));
    phy.SetChannel( channel.Create());
    WifiMacHelper mac;
    mac.SetType( "ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install( phy, mac, nodes);
    // Fixed Wi-Fi streams.
    wifi.AssignStreams( devices, 10000);
    return devices;
}
// MOBILITY
static void
InstallMobility( NodeContainer& nodes, const Scenario& s)
{
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    for (const Vector& position : s.positions)
    {
        allocator->Add( position);
    }
MobilityHelper mobility;
mobility.SetPositionAllocator( allocator);
mobility.SetMobilityModel( "ns3::RandomWalk2dMobilityModel", "Mode", StringValue( "Time"), "Time", TimeValue( MilliSeconds(500)), "Speed", StringValue( "ns3::ConstantRandomVariable[Constant=" + std::to_string( s.speed) + "]"), "Bounds", RectangleValue( Rectangle( 0.0, s.terrain, 0.0, s.terrain)));
mobility.Install( nodes);
// Same mobility streams for every protocol
// within the same scenario.
mobility.AssignStreams( nodes, 0);
}
// ROUTING
static void
InstallRouting( NodeContainer& nodes, const Scenario& s, const std::string& protocol)
{
    InternetStackHelper internet;
    if (protocol == "AODV-AERS1" || protocol == "AODV-AERS2" || protocol == "AODV" || protocol == "AODV-AERS3")
    {
        AodvHelper aodv;
        ConfigureAodv( aodv, s, protocol);
        internet.SetRoutingHelper( aodv);
        internet.Install( nodes);
    }
else if (protocol == "OLSR")
{
    OlsrHelper olsr;
    internet.SetRoutingHelper( olsr);
    internet.Install( nodes);
}
else if (protocol == "DSDV")
{
    DsdvHelper dsdv;
    internet.SetRoutingHelper( dsdv);
    internet.Install( nodes);
}
}
// RED QUEUE
static void
InstallRedQueue( const Scenario& s, const NetDeviceContainer& devices)
{
    TrafficControlHelper trafficControl;
    double lInterm =1.0 /
    s.dropProbability;
    trafficControl.SetRootQueueDisc( "ns3::RedQueueDisc", "MaxSize", StringValue( std::to_string( s.bufferSize) + "p"), "MinTh", DoubleValue( s.threshMin), "MaxTh", DoubleValue( s.threshMax), "MeanPktSize", UintegerValue( s.packetSize), "LInterm", DoubleValue( lInterm), "Gentle", BooleanValue( true), "Wait", BooleanValue( true), "LinkBandwidth", DataRateValue( DataRate( "11Mbps")), "LinkDelay", TimeValue( MilliSeconds(1)), "UseEcn", BooleanValue( false));
    trafficControl.Install( devices);
}
// APPLICATION SINK
static ApplicationContainer
InstallSink( NodeContainer& nodes, uint32_t destination)
{
    PacketSinkHelper sink( "ns3::UdpSocketFactory", InetSocketAddress( Ipv4Address::GetAny(), APP_PORT));
    ApplicationContainer sinkApp = sink.Install( nodes.Get( destination));
    sinkApp.Start( Seconds(0.0));
    sinkApp.Stop( Seconds(SIM_TIME));
    return sinkApp;
}
// APPLICATION SOURCE
static ApplicationContainer
InstallSource( NodeContainer& nodes, uint32_t source, Ipv4Address destinationAddress, uint64_t trafficRate, uint32_t packetSize)
{
    OnOffHelper sourceHelper( "ns3::UdpSocketFactory", InetSocketAddress( destinationAddress, APP_PORT));
    sourceHelper.SetConstantRate( DataRate( trafficRate), packetSize);
    ApplicationContainer sourceApp = sourceHelper.Install( nodes.Get( source));
    sourceApp.Start( Seconds(APP_START));
    sourceApp.Stop( Seconds(APP_STOP));
    return sourceApp;
}
// METRIC EXTRACTION
static ProtocolResult
ExtractMetrics( FlowMonitorHelper& flowHelper, Ptr<FlowMonitor> monitor, Ipv4Address destinationAddress)
{
    ProtocolResult result;
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =DynamicCast<Ipv4FlowClassifier>(
    flowHelper.GetClassifier());
    if (classifier == nullptr)
    {
        return result;
    }
FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
// CHANGED: accumulate delay properly instead of overwriting
// with only the last matching flow's average.
double delayMsSum = 0.0;
for (const auto& entry : stats)
{
    FlowId flowId = entry.first;
    Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow( flowId);
    // Only our application destination.
    if (tuple.destinationAddress != destinationAddress)
    {
        continue;
    }
// Only UDP.
if (tuple.protocol != 17)
{
    continue;
}
// Only our application.
if (tuple.destinationPort != APP_PORT)
{
    continue;
}
const FlowMonitor::FlowStats& flowStats = entry.second;
result.txPackets += static_cast<uint64_t>(
flowStats.txPackets);
result.rxPackets += static_cast<uint64_t>(
flowStats.rxPackets);
result.throughputMbps += static_cast<double>(
flowStats.rxBytes) *
8.0 /
((APP_STOP - APP_START) *
1000000.0);
// Sum of per-packet delays (ms) across all matching flows;
// divided by total rxPackets after the loop for a correct
// weighted average instead of "last flow wins".
delayMsSum +=flowStats.delaySum.GetSeconds() *
1000.0;
}
if (result.txPackets > 0)
{
    result.pdr = static_cast<double>(
    result.rxPackets) /
    static_cast<double>(
    result.txPackets);
}
if (result.rxPackets > 0)
{
    result.delayMs =delayMsSum /
    static_cast<double>(
    result.rxPackets);
}
else
{
    result.delayMs = -1.0;
}
return result;
}
// RUN ONE PROTOCOL
static ProtocolResult
RunProtocol( const Scenario& s, const std::string& protocol)
{
    RngSeedManager::SetSeed( RNG_SEED);
    RngSeedManager::SetRun( s.id);
    NodeContainer nodes;
    nodes.Create( s.nodes);
    // SAME INITIAL POSITIONS
    InstallMobility( nodes, s);
    // SAME WIFI
    NetDeviceContainer devices =InstallNetworkDevices( nodes);
    // ROUTING
    InstallRouting( nodes, s, protocol);
    // AODV-RED
    if (protocol == "AODV-AERS3")
    {
        InstallRedQueue( s, devices);
    }
// IP ADDRESSING
Ipv4AddressHelper address;
address.SetBase( "10.0.0.0", "255.255.255.0");
Ipv4InterfaceContainer interfaces = address.Assign( devices);
Ipv4Address destinationAddress = interfaces.GetAddress( s.destination);
// APPLICATIONS
ApplicationContainer sink =InstallSink( nodes, s.destination);
ApplicationContainer source =InstallSource( nodes, s.source, destinationAddress, s.trafficRate, s.packetSize);
(void)sink;
(void)source;
// FLOW MONITOR
FlowMonitorHelper flowHelper;
Ptr<FlowMonitor> monitor = flowHelper.InstallAll();
// SIMULATION
Simulator::Stop( Seconds(SIM_TIME));
Simulator::Run();
// METRICS
ProtocolResult result =ExtractMetrics( flowHelper, monitor, destinationAddress);
Simulator::Destroy();
return result;
}
// WINNER: THROUGHPUT
static std::string
FindThroughputWinner( const std::vector<ProtocolRun>& results, double& value)
{
    double best = -1.0;
    std::string winner ="NONE";
    for (const auto& protocol : results)
    {
        if (protocol.result.throughputMbps > best)
        {
            best = protocol.result.throughputMbps;
            winner = protocol.name;
        }
}
if (best <= 0.0)
{
    value = 0.0;
    return "NONE";
}
value = best;
return winner;
}
// WINNER: PDR
static std::string
FindPdrWinner( const std::vector<ProtocolRun>& results, double& value)
{
    double best = -1.0;
    std::string winner ="NONE";
    for (const auto& protocol : results)
    {
        if (protocol.result.pdr > best)
        {
            best = protocol.result.pdr;
            winner = protocol.name;
        }
}
if (best <= 0.0)
{
    value = 0.0;
    return "NONE";
}
value = best;
return winner;
}
// WINNER: DELAY
static std::string
FindDelayWinner( const std::vector<ProtocolRun>& results, double& value)
{
    double best = std::numeric_limits<double>::max();
    std::string winner ="NONE";
    for (const auto& protocol : results)
    {
        // Delay is undefined if no packets were received.
        if (protocol.result.rxPackets == 0)
        {
            continue;
        }
    if (protocol.result.delayMs < 0.0)
    {
        continue;
    }
if (protocol.result.delayMs < best)
{
    best = protocol.result.delayMs;
    winner = protocol.name;
}
}
if (winner == "NONE")
{
    value = -1.0;
    return "NONE";
}
value = best;
return winner;
}
// CSV HEADER
static void
WriteCsvHeader( std::ofstream& csv)
{
    csv
    << "Scenario_ID,"
    << "Topology,"
    << "Node_Count,"
    << "Node_Density_nodes_per_km2,"
    << "Terrain_Size_m,"
    << "Node_Speed_mps,"
    << "Traffic_Rate_bps,"
    << "Packet_Size_bytes,"
    << "TTL,"
    << "Wait_Time_s,"
    << "Retry_Attempts,"
    << "Route_Expiry_s,"
    << "Buffer_Size_packets,"
    << "Thresh_Min,"
    << "Thresh_Max,"
    << "Drop_Probability,"
    << "Source_Node,"
    << "Destination_Node,"
    << "Initial_Hops,";
    const std::vector<std::string> protocols ={
        "AODV-AERS1",
        "AODV-AERS2",
        "AODV-AERS3",
        "OLSR",
        "DSDV",
        "AODV"
    };
for (const std::string& protocol : protocols)
{
    csv
    << protocol
    << "_Throughput_Mbps,"
    << protocol
    << "_PDR,"
    << protocol
    << "_E2E_Delay_ms,"
    << protocol
    << "_TxPackets,"
    << protocol
    << "_RxPackets,";
}
// THREE TARGET CLASSES
csv
<< "Optimal_Protocol_Throughput,"
<< "Optimal_Protocol_PDR,"
<< "Optimal_Protocol_Delay,"
<< "Winning_Throughput_Mbps,"
<< "Winning_PDR,"
<< "Winning_Delay_ms\n";
}
// WRITE CSV ROW
static void
WriteCsvRow( std::ofstream& csv, const Scenario& s, const std::vector<ProtocolRun>& results, const std::string& throughputWinner, 
    const std::string& pdrWinner, const std::string& delayWinner, double winningThroughput, double winningPdr, double winningDelay)
{
    csv
    << std::fixed
    << std::setprecision(6);
    csv
    << s.id << ","
    << s.topology << ","
    << s.nodes << ","
    << s.nodeDensity << ","
    << s.terrain << ","
    << s.speed << ","
    << s.trafficRate << ","
    << s.packetSize << ","
    << s.ttl << ","
    << s.waitTime << ","
    << s.retryAttempts << ","
    << s.routeExpiry << ","
    << s.bufferSize << ","
    << s.threshMin << ","
    << s.threshMax << ","
    << s.dropProbability << ","
    << s.source << ","
    << s.destination << ","
    << s.initialHops << ",";
    // PROTOCOL METRICS
    for (const auto& protocol : results)
    {
        csv
        << protocol.result.throughputMbps
        << ","
        << protocol.result.pdr
        << ","
        << protocol.result.delayMs
        << ","
        << protocol.result.txPackets
        << ","
        << protocol.result.rxPackets
        << ",";
    }
// TARGET LABELS
csv
<< throughputWinner
<< ","
<< pdrWinner
<< ","
<< delayWinner
<< ","
<< winningThroughput
<< ","
<< winningPdr
<< ","
<< winningDelay
<< "\n";
}
// MAIN
int
main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    // SCENARIO GENERATION RNG
    std::mt19937 scenarioRng( 20260829);
    std::vector<Scenario> scenarios;
    scenarios.reserve( NUM_SCENARIOS);
    // GENERATE 1000 CONFIGURATIONS
    for (uint32_t i = 0; i < NUM_SCENARIOS; ++i)
    {
        // Random topology family.
        uint32_t topologyType = RandUInt( scenarioRng, 0, 6);
        Scenario scenario = GenerateScenario( i + 1, topologyType, scenarioRng);
        scenarios.push_back( std::move( scenario));
    }
std::shuffle( scenarios.begin(), scenarios.end(), scenarioRng);
// CSV
std::ofstream csv( "manet_dataset.csv");
if (!csv.is_open())
{
    std::cerr
    << "ERROR: Could not open "
    << "manet_dataset.csv\n";
    return 1;
}
WriteCsvHeader( csv);
// EXACT PROTOCOL LIST
const std::vector<std::string> protocols ={
    "AODV-AERS1",
    "AODV-AERS2",
    "AODV-AERS3",
    "OLSR",
    "DSDV",
    "AODV"
};
// RUN ALL SCENARIOS
for (uint32_t index = 0; index < NUM_SCENARIOS; ++index)
{
    const Scenario& s = scenarios[index];
    // CONSOLE OUTPUT
    std::cout
    << "\n"
    << "==========================================\n"
    << "SCENARIO "
    << index + 1
    << " / "
    << NUM_SCENARIOS
    << "\n"
    << "==========================================\n";
    std::cout
    << "Nodes: "
    << s.nodes
    << "\n";
    std::cout
    << "Terrain: "
    << std::fixed
    << std::setprecision(2)
    << s.terrain
    << " x "
    << s.terrain
    << "\n";
    std::cout
    << "Speed: "
    << std::setprecision(5)
    << s.speed
    << " m/s\n";
    std::cout
    << "Traffic: "
    << s.trafficRate
    << " bps\n";
    std::cout
    << "Packet size: "
    << s.packetSize
    << " bytes\n";
    std::cout
    << "Topology: "
    << s.topology
    << "\n";
    std::cout
    << "Initial hops: "
    << s.initialHops
    << "\n";
    // RUN ALL 7 PROTOCOLS
    std::vector<ProtocolRun> results;
    results.reserve( protocols.size());
    for (const std::string& protocol : protocols)
    {
        std::cout
        << "    Running "
        << protocol
        << "...\n";
        ProtocolResult result =RunProtocol( s, protocol);
        std::cout
        << "        Throughput: "
        << std::fixed
        << std::setprecision(3)
        << result.throughputMbps
        << " Mbps\n";
        std::cout
        << "        PDR: "
        << std::setprecision(4)
        << result.pdr
        << "\n";
        std::cout
        << "        E2E: "
        << std::setprecision(3)
        << result.delayMs
        << " ms\n";
        if (result.rxPackets == 0)
        {
            std::cout
            << "        [WARNING] 0 packets received for "
            << protocol
            << " on scenario "
            << s.id
            << " -- check convergence/timing for this protocol.\n";
        }
    results.push_back( { protocol, result });
}
// FIND THREE INDEPENDENT WINNERS
double winningThroughput = 0.0;
double winningPdr = 0.0;
double winningDelay = -1.0;
std::string throughputWinner =FindThroughputWinner( results, winningThroughput);
std::string pdrWinner =FindPdrWinner( results, winningPdr);
std::string delayWinner =FindDelayWinner( results, winningDelay);
// CONSOLE WINNERS
std::cout
<< "\n"
<< "WINNERS:\n";
std::cout
<< "Throughput: "
<< throughputWinner
<< "\n";
std::cout
<< "PDR: "
<< pdrWinner
<< "\n";
std::cout
<< "E2E Delay: "
<< delayWinner
<< "\n";
// SAVE CSV ROW
WriteCsvRow( csv, s, results, throughputWinner, pdrWinner, delayWinner, winningThroughput, winningPdr, winningDelay);
csv.flush();
}
csv.close();
// COMPLETE
std::cout
<< "\n"
<< "==========================================\n"
<< "DATASET GENERATION COMPLETE\n"
<< "==========================================\n"
<< "Scenarios: "
<< NUM_SCENARIOS
<< "\n"
<< "Protocols per scenario: "
<< protocols.size()
<< "\n"
<< "Total simulations: "
<< NUM_SCENARIOS *
protocols.size()
<< "\n"
<< "Output: manet_dataset.csv\n";
return 0;
}
