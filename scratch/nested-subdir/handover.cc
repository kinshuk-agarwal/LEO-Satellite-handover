/*
 * handover.cc — Starlink LEO Satellite Handover Simulation
 * =========================================================
 * Nodes:
 *   0-4  : LEO satellites
 *   5    : Hyderabad (transmitter)
 *   6    : Mumbai    (receiver)
 *
 * Reads topology.json produced by fixed orbit.py which now
 * selects satellites visible from BOTH cities simultaneously.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace ns3;

static const double KU_FREQ_GHZ    = 12.0;
static const double KA_FREQ_GHZ    = 26.5;
static const double BEAM_HALF_DEG  = 26.0;
static const double SPEED_OF_LIGHT = 299792.458;  // km/s

NodeContainer nodes;

std::vector<std::vector<Ptr<PointToPointChannel>>> groundChannels(
    2, std::vector<Ptr<PointToPointChannel>>(5));

std::vector<Ptr<PointToPointChannel>> islChannels;

std::vector<Ptr<PacketSink>> hydSinks;
std::vector<Ptr<PacketSink>> mumSinks;

uint64_t lastHydRx = 0, lastMumRx = 0;

int      bestSat_Hyd    = -2;
int      bestSat_Mum    = -2;
uint32_t gndHO_Hyd      = 0;
uint32_t gndHO_Mum      = 0;
uint32_t islHO_count    = 0;
uint32_t linkFail_count = 0;
uint32_t softHO_count   = 0;
uint32_t hardHO_count   = 0;

std::vector<std::string> satNames;

// ── Throughput logger ─────────────────────────────────────────────────────────
void LogThroughput()
{
    uint64_t hyd_total = 0, mum_total = 0;
    for (auto& s : hydSinks) if (s) hyd_total += s->GetTotalRx();
    for (auto& s : mumSinks) if (s) mum_total += s->GetTotalRx();

    double tpHyd = (hyd_total - lastHydRx) * 8.0 / 1e6;
    double tpMum = (mum_total - lastMumRx) * 8.0 / 1e6;
    lastHydRx = hyd_total;
    lastMumRx = mum_total;

    auto nameOf = [](int idx) -> std::string {
        return (idx >= 0 && idx < (int)satNames.size()) ? satNames[idx] : "NONE";
    };

    std::cout << std::fixed << std::setprecision(2)
              << "[t=" << std::setw(7) << Simulator::Now().GetSeconds() << "s]"
              << "  HYD->MUM: " << std::setw(7) << tpMum << " Mbps"
              << "  MUM->HYD: " << std::setw(7) << tpHyd << " Mbps"
              << "  | HydSat: " << std::setw(16) << nameOf(bestSat_Hyd)
              << "  MumSat: "   << std::setw(16) << nameOf(bestSat_Mum)
              << "  | GHO: " << (gndHO_Hyd + gndHO_Mum)
              << " (S:" << softHO_count << "/H:" << hardHO_count << ")"
              << "  ISHO: " << islHO_count
              << "  FAIL: " << linkFail_count
              << "\n";

    Simulator::Schedule(Seconds(5.0), &LogThroughput);
}

// ── Ground link update ────────────────────────────────────────────────────────
void UpdateGroundLink(uint32_t gsIdx, uint32_t satIdx,
                      bool visible, double distKm)
{
    Ptr<PointToPointChannel> ch = groundChannels[gsIdx][satIdx];
    if (!visible)
        ch->SetAttribute("Delay", TimeValue(Seconds(1000.0)));
    else
        ch->SetAttribute("Delay", TimeValue(Seconds(distKm / SPEED_OF_LIGHT)));
}

// ── ISL link update ───────────────────────────────────────────────────────────
void UpdateIslLink(uint32_t idx, double distKm, bool feasible)
{
    Ptr<PointToPointChannel> ch = islChannels[idx];
    if (!feasible)
        ch->SetAttribute("Delay", TimeValue(Seconds(1000.0)));
    else
        ch->SetAttribute("Delay", TimeValue(Seconds(distKm / SPEED_OF_LIGHT)));
}

// ── Link failure — fires ONCE ─────────────────────────────────────────────────
void TriggerLinkFailure(uint32_t gsIdx, uint32_t satIdx,
                        std::string satName, std::string gsName)
{
    linkFail_count++;
    groundChannels[gsIdx][satIdx]->SetAttribute("Delay", TimeValue(Seconds(1000.0)));

    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════╗\n"
              << "║  LINK FAILURE #" << linkFail_count
              << "  at t=" << std::fixed << std::setprecision(1)
              << Simulator::Now().GetSeconds() << "s\n"
              << "║  " << satName << " <-> " << gsName << " FAILED\n"
              << "║  Ku-band " << KU_FREQ_GHZ << " GHz link hard drop\n"
              << "║  OLSR reroutes after ~6s hello timeout\n"
              << "╚══════════════════════════════════════════════════════╝\n\n";
}

// ── Link restore — fires ONCE ─────────────────────────────────────────────────
void RestoreLink(uint32_t gsIdx, uint32_t satIdx, double distKm)
{
    groundChannels[gsIdx][satIdx]->SetAttribute(
        "Delay", TimeValue(Seconds(distKm / SPEED_OF_LIGHT)));
    std::cout << "[t=" << std::fixed << std::setprecision(1)
              << Simulator::Now().GetSeconds()
              << "s] Link restored: GS[" << gsIdx << "] <-> Sat[" << satIdx << "]\n";
}

// ── Ground handover — ho_type from orbit.py ──────────────────────────────────
void CheckGroundHandover(uint32_t gsIdx, int newBestSat,
                         std::string hoType, std::string gsName)
{
    int&      curBest = (gsIdx == 0) ? bestSat_Hyd : bestSat_Mum;
    uint32_t& hoCount = (gsIdx == 0) ? gndHO_Hyd   : gndHO_Mum;

    if (newBestSat == curBest) return;

    if (curBest == -2)
    {
        curBest = newBestSat;
        std::string name = (newBestSat >= 0) ? satNames[newBestSat] : "NONE";
        std::cout << "[t=0s] " << gsName << " initial satellite: " << name << "\n";
        return;
    }

    std::string from = (curBest    >= 0) ? satNames[curBest]    : "NONE";
    std::string to   = (newBestSat >= 0) ? satNames[newBestSat] : "NONE";
    hoCount++;

    if (hoType == "SOFT") softHO_count++;
    else                  hardHO_count++;

    std::string label = (hoType == "SOFT")
        ? "SOFT HO (make-before-break)"
        : "HARD HO (break-before-make)";

    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════╗\n"
              << "║  " << gsName << " GROUND HANDOVER #" << hoCount
              << "  at t=" << std::fixed << std::setprecision(1)
              << Simulator::Now().GetSeconds() << "s\n"
              << "║  Type  : " << label << "\n"
              << "║  Switch: " << from << "  -->  " << to << "\n"
              << "║  Freq  : Ku-band " << KU_FREQ_GHZ << " GHz"
              << "  Beam: " << (BEAM_HALF_DEG * 2) << " deg total\n"
              << "╚══════════════════════════════════════════════════════╝\n\n";

    curBest = newBestSat;
}

// ── ISL path change ───────────────────────────────────────────────────────────
void ReportIslHandover(std::string pair, std::string oldPath, std::string newPath)
{
    islHO_count++;
    std::cout << "\n"
              << "╔══════════════════════════════════════════════════════╗\n"
              << "║  ISL HANDOVER #" << islHO_count
              << "  at t=" << std::fixed << std::setprecision(1)
              << Simulator::Now().GetSeconds() << "s\n"
              << "║  Pair    : " << pair << "\n"
              << "║  Old path: " << oldPath << "\n"
              << "║  New path: " << newPath << "\n"
              << "║  Freq    : Ka-band " << KA_FREQ_GHZ << " GHz ISL\n"
              << "╚══════════════════════════════════════════════════════╝\n\n";
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char *argv[])
{
    nodes.Create(7);  // 0-4 sats | 5 Hyderabad | 6 Mumbai

    PointToPointHelper islP2p;
    islP2p.SetDeviceAttribute("DataRate",  StringValue("50Mbps"));
    islP2p.SetChannelAttribute("Delay",    StringValue("5ms"));

    PointToPointHelper groundP2p;
    groundP2p.SetDeviceAttribute("DataRate",  StringValue("100Mbps"));
    groundP2p.SetChannelAttribute("Delay",    StringValue("1000s"));

    NetDeviceContainer allDevices;

    // ISL full mesh (10 links)
    for (int i = 0; i < 5; i++)
        for (int j = i + 1; j < 5; j++)
        {
            auto lnk = islP2p.Install(nodes.Get(i), nodes.Get(j));
            allDevices.Add(lnk);
            islChannels.push_back(
                DynamicCast<PointToPointChannel>(lnk.Get(0)->GetChannel()));
        }

    // Hyderabad ground links (node 5 <-> each sat)
    for (int i = 0; i < 5; i++)
    {
        auto lnk = groundP2p.Install(nodes.Get(5), nodes.Get(i));
        allDevices.Add(lnk);
        groundChannels[0][i] =
            DynamicCast<PointToPointChannel>(lnk.Get(0)->GetChannel());
    }

    // Mumbai ground links (node 6 <-> each sat)
    for (int i = 0; i < 5; i++)
    {
        auto lnk = groundP2p.Install(nodes.Get(6), nodes.Get(i));
        allDevices.Add(lnk);
        groundChannels[1][i] =
            DynamicCast<PointToPointChannel>(lnk.Get(0)->GetChannel());
    }

    // OLSR routing
    OlsrHelper olsr;
    InternetStackHelper stack;
    stack.SetRoutingHelper(olsr);
    stack.Install(nodes);

    // IP addressing
    Ipv4AddressHelper addr;
    addr.SetBase("10.0.0.0", "255.255.255.0");
    int devIdx = 0;

    for (int k = 0; k < 10; k++)   // ISL subnets
    {
        NetDeviceContainer pr;
        pr.Add(allDevices.Get(devIdx));
        pr.Add(allDevices.Get(devIdx + 1));
        addr.Assign(pr); addr.NewNetwork(); devIdx += 2;
    }
    for (int i = 0; i < 5; i++)    // Hyderabad subnets
    {
        NetDeviceContainer pr;
        pr.Add(allDevices.Get(devIdx));
        pr.Add(allDevices.Get(devIdx + 1));
        addr.Assign(pr); addr.NewNetwork(); devIdx += 2;
    }
    for (int i = 0; i < 5; i++)    // Mumbai subnets
    {
        NetDeviceContainer pr;
        pr.Add(allDevices.Get(devIdx));
        pr.Add(allDevices.Get(devIdx + 1));
        addr.Assign(pr); addr.NewNetwork(); devIdx += 2;
    }

    Ipv4Address hydIp = nodes.Get(5)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    Ipv4Address mumIp = nodes.Get(6)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    std::cout << "Hyderabad IP : " << hydIp << "\n";
    std::cout << "Mumbai IP    : " << mumIp << "\n";
    std::cout << "Freq model   : Ku " << KU_FREQ_GHZ << " GHz | Ka "
              << KA_FREQ_GHZ << " GHz | Beam " << (BEAM_HALF_DEG*2) << " deg\n\n";

    // ── Applications ──────────────────────────────────────────────────────────
    // HYD->MUM 10 Mbps (port 9)  |  MUM->HYD 5 Mbps (port 10)
    // Start at t=40s (OLSR convergence ~30s)

    mumSinks.resize(1, nullptr);
    {
        PacketSinkHelper sk("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), 9));
        auto app = sk.Install(nodes.Get(6));
        app.Start(Seconds(0.0));
        mumSinks[0] = DynamicCast<PacketSink>(app.Get(0));
    }
    hydSinks.resize(1, nullptr);
    {
        PacketSinkHelper sk("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), 10));
        auto app = sk.Install(nodes.Get(5));
        app.Start(Seconds(0.0));
        hydSinks[0] = DynamicCast<PacketSink>(app.Get(0));
    }
    {
        OnOffHelper onoff("ns3::UdpSocketFactory",
            InetSocketAddress(mumIp, 9));
        onoff.SetConstantRate(DataRate("10Mbps"));
        onoff.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        auto app = onoff.Install(nodes.Get(5));
        app.Start(Seconds(40.0)); app.Stop(Seconds(1800.0));
    }
    {
        OnOffHelper onoff("ns3::UdpSocketFactory",
            InetSocketAddress(hydIp, 10));
        onoff.SetConstantRate(DataRate("5Mbps"));
        onoff.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        auto app = onoff.Install(nodes.Get(6));
        app.Start(Seconds(40.0)); app.Stop(Seconds(1800.0));
    }

    Simulator::Schedule(Seconds(5.0), &LogThroughput);

    // ── Load topology.json ─────────────────────────────────────────────────────
    std::ifstream file("../sat-project/topology.json");
    if (!file.is_open())
    {
        std::cerr << "ERROR: Cannot open ../sat-project/topology.json\n";
        return 1;
    }
    Json::Value root;
    file >> root;

    if (root.isMember("sat_names"))
        for (unsigned int i = 0; i < root["sat_names"].size(); i++)
            satNames.push_back(root["sat_names"][i].asString());
    while ((int)satNames.size() < 5)
        satNames.push_back("SAT-" + std::to_string(satNames.size()));

    std::cout << "Satellites:";
    for (auto& n : satNames) std::cout << " " << n;
    std::cout << "\n\n";

    const Json::Value& snaps = root["snapshots"];
    std::cout << snaps.size() << " snapshots loaded\n\n";

    const double RESTORE_DELAY_KM = 550.0;

    for (unsigned int s = 0; s < snaps.size(); s++)
    {
        const Json::Value& snap = snaps[s];
        uint32_t simTime = snap["time"].asUInt();

        // Hyderabad ground links
        if (snap.isMember("hyd_links"))
        {
            const Json::Value& links = snap["hyd_links"];
            for (unsigned int g = 0; g < links.size(); g++)
            {
                bool   vis  = links[g]["visible"].asBool();
                double dist = links[g]["distance_km"].asDouble();
                Simulator::Schedule(Seconds(simTime),
                    &UpdateGroundLink, (uint32_t)0, g, vis, dist);
            }
        }

        // Mumbai ground links
        if (snap.isMember("mum_links"))
        {
            const Json::Value& links = snap["mum_links"];
            for (unsigned int g = 0; g < links.size(); g++)
            {
                bool   vis  = links[g]["visible"].asBool();
                double dist = links[g]["distance_km"].asDouble();
                Simulator::Schedule(Seconds(simTime),
                    &UpdateGroundLink, (uint32_t)1, g, vis, dist);
            }
        }

        // ISL links
        if (snap.isMember("isl_links"))
        {
            const Json::Value& links = snap["isl_links"];
            for (unsigned int l = 0; l < links.size(); l++)
            {
                double dist   = links[l]["distance_km"].asDouble();
                bool feasible = links[l]["feasible"].asBool();
                Simulator::Schedule(Seconds(simTime),
                    &UpdateIslLink, l, dist, feasible);
            }
        }

        // Ground handover events (with ho_type)
        if (snap.isMember("handover_events"))
        {
            const Json::Value& evts = snap["handover_events"];
            for (unsigned int e = 0; e < evts.size(); e++)
            {
                int    gsIdx = evts[e]["gs_idx"].asInt();
                int    toSat = evts[e]["to_sat"].asInt();
                std::string ht = evts[e].isMember("ho_type")
                                 ? evts[e]["ho_type"].asString() : "HARD";
                std::string gn = evts[e].isMember("gs_name")
                                 ? evts[e]["gs_name"].asString() : "GS";
                Simulator::Schedule(Seconds(simTime),
                    &CheckGroundHandover, (uint32_t)gsIdx, toSat, ht, gn);
            }
        }

        // ISL handover events
        if (snap.isMember("isl_ho_events"))
        {
            const Json::Value& evts = snap["isl_ho_events"];
            for (unsigned int e = 0; e < evts.size(); e++)
            {
                std::string pair = evts[e]["pair"].asString();
                std::ostringstream oldSS, newSS;
                for (unsigned int p = 0; p < evts[e]["old_path"].size(); p++)
                {
                    if (p > 0) oldSS << " -> ";
                    oldSS << evts[e]["old_path"][p].asString();
                }
                for (unsigned int p = 0; p < evts[e]["new_path"].size(); p++)
                {
                    if (p > 0) newSS << " -> ";
                    newSS << evts[e]["new_path"][p].asString();
                }
                Simulator::Schedule(Seconds(simTime),
                    &ReportIslHandover, pair, oldSS.str(), newSS.str());
            }
        }

        // Link failure starts (fire ONCE)
        if (snap.isMember("failure_starts"))
        {
            const Json::Value& fs = snap["failure_starts"];
            for (unsigned int f = 0; f < fs.size(); f++)
            {
                uint32_t    satIdx  = fs[f]["sat_idx"].asUInt();
                uint32_t    gsIdx   = fs[f]["gs_idx"].asUInt();
                std::string satName = fs[f]["sat_name"].asString();
                std::string gsName  = fs[f]["gs_name"].asString();
                Simulator::Schedule(Seconds(simTime),
                    &TriggerLinkFailure, gsIdx, satIdx, satName, gsName);
            }
        }

        // Link failure ends (fire ONCE)
        if (snap.isMember("failure_ends"))
        {
            const Json::Value& fe = snap["failure_ends"];
            for (unsigned int f = 0; f < fe.size(); f++)
            {
                uint32_t satIdx = fe[f]["sat_idx"].asUInt();
                uint32_t gsIdx  = fe[f]["gs_idx"].asUInt();
                Simulator::Schedule(Seconds(simTime),
                    &RestoreLink, gsIdx, satIdx, RESTORE_DELAY_KM);
            }
        }
    }

    // ── Run ───────────────────────────────────────────────────────────────────
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    std::cout << "=== Simulation Starting ===\n";
    std::cout << "HYD (node 5) --[sat backbone]--> MUM (node 6)\n";
    std::cout << "Traffic: 10 Mbps HYD->MUM | 5 Mbps MUM->HYD\n";
    std::cout << "Freq: Ku " << KU_FREQ_GHZ << " GHz | Ka " << KA_FREQ_GHZ
              << " GHz | Beam " << (BEAM_HALF_DEG*2) << " deg\n";
    std::cout << "Satellites selected for dual visibility (both cities)\n\n";

    Simulator::Stop(Seconds(1800.0));
    Simulator::Run();

    std::cout << "\n=== SIMULATION COMPLETE ===\n";
    std::cout << "Hyderabad GND HO : " << gndHO_Hyd   << "\n";
    std::cout << "Mumbai    GND HO : " << gndHO_Mum   << "\n";
    std::cout << "Soft handovers   : " << softHO_count << "  (make-before-break)\n";
    std::cout << "Hard handovers   : " << hardHO_count << "  (break-before-make)\n";
    std::cout << "ISL path changes : " << islHO_count  << "\n";
    std::cout << "Link failures    : " << linkFail_count << "\n";

    monitor->SerializeToXmlFile("flows.xml", true, true);
    std::cout << "Flow stats: flows.xml\n";

    Simulator::Destroy();
    return 0;
}