/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 University of Kansas
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
  You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Justin Rohrer <rohrej@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/vanet-topology.h"

using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("vanet-routing-compare");

class VanetRoutingExperiment
{
public:
  VanetRoutingExperiment ();
  void Run ();
  void CommandSetup (int argc, char **argv);

  static int wavePktSendCount;
  static int wavePktReceiveCount;
  static int wavePktInCoverageReceiveCount;
  static int wavePktExpectedReceiveCount;
  static NodeContainer m_adhocTxNodes;
  static double m_txSafetyRangeSq;

private:
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);
  void ReceivePacket (Ptr<Socket> socket);
  void ReceiveWavePacket (Ptr<Socket> socket);
  void CheckThroughput ();
  void SetupLogFile ();
  void SetupLogging ();
  void ConfigureDefaults ();
  void SetupAdhocMobilityNodes ();
  void SetupAdhocDevices ();
  void SetupRouting ();
  void AssignIpAddresses ();
  void SetupWaveMessages ();
  void SetupRoutingMessages ();
  void SetupScenario ();
  void WriteCsvHeader ();

  uint32_t port;
  uint32_t bytesTotal;
  uint32_t packetsReceived;

  std::string m_CSVfileName;
  int m_nSinks;
  std::string m_protocolName;
  double m_txp;
  bool m_traceMobility;
  uint32_t m_protocol;

  // antenna height
  double m_hEnb;
  double m_hUe;

  // indoor / outdoor selections
  // for propagation
  bool m_enbIndoor;
  bool m_ueIndoor;

  uint32_t m_lossModel;
  std::string m_lossModelName;

  std::string m_phyMode;
  uint32_t m_80211mode;

  std::string m_traceFile;
  std::string m_logFile;
  uint32_t m_mobility;
  uint32_t m_nNodes;
  double m_TotalTime;
  std::string m_rate;
  std::string m_phyMode_b;
  std::string m_tr_name;
  int m_nodeSpeed; //in m/s
  int m_nodePause; //in s
  uint32_t m_wavePacketSize; // bytes
  uint32_t m_numWavePackets;
  double m_waveInterval; // seconds
  int m_verbose;
  std::ofstream m_os;
  //NodeContainer m_adhocTxNodes;
  NetDeviceContainer m_adhocTxDevices;
  Ipv4InterfaceContainer m_adhocTxInterfaces;
  uint32_t m_scenario;
  int m_flowmon;
  double m_gpsAccuracyNs;

  int m_loadBuildings;
};

int VanetRoutingExperiment::wavePktSendCount = 0;
int VanetRoutingExperiment::wavePktReceiveCount = 0;
int VanetRoutingExperiment::wavePktInCoverageReceiveCount = 0;
int VanetRoutingExperiment::wavePktExpectedReceiveCount = 0;
NodeContainer VanetRoutingExperiment::m_adhocTxNodes;
double VanetRoutingExperiment::m_txSafetyRangeSq = 145.0 * 145.0;

VanetRoutingExperiment::VanetRoutingExperiment ()
  : port (9),
    bytesTotal (0),
    packetsReceived (0),
    m_CSVfileName ("vanet-routing.output.csv"),
    m_nSinks (10),
    m_protocolName ("protocol"),
    m_txp (7.5),
    m_traceMobility (false),
    m_protocol (2), // AODV
    m_hEnb (2.0),
    m_hUe (2.0),
    m_enbIndoor (false),
    m_ueIndoor (false),
    m_lossModel (2), // ITU R-1441
    m_lossModelName (""),
    m_phyMode ("OfdmRate6MbpsBW10MHz"),
    m_80211mode (1),  // 1=802.11p
    m_traceFile ("./scratch/low_ct-unterstrass-1day.filt.5.adj.mov"),
    m_logFile ("low_ct-unterstrass-1day.filt.5.adj.log"),
    m_mobility (1),
    m_nNodes (156),
    m_TotalTime(300.01),
    m_rate ("2048bps"),
    m_phyMode_b ("DsssRate11Mbps"),
    m_tr_name ("vanet-routing-compare"),
    m_nodeSpeed (20),
    m_nodePause (0),
    m_wavePacketSize (200),
    m_numWavePackets (1),
    m_waveInterval (0.1),
    m_verbose (0),
    m_scenario (1),
    m_flowmon (1),
    m_gpsAccuracyNs (10000),
    m_loadBuildings (0)
{
}

// Prints actual position and velocity when a course change event occurs
static void
CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition (); // Get position
  Vector vel = mobility->GetVelocity (); // Get velocity

  pos.z = 1.5;

  // Prints position and velocities
  *os << Simulator::Now () << " POS: x=" << pos.x << ", y=" << pos.y
      << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
      << ", z=" << vel.z << std::endl;
}


static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet)
{
  SocketAddressTag tag;
  bool found;
  found = packet->PeekPacketTag (tag);
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (found)
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (tag.GetAddress ());
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}

void
VanetRoutingExperiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      NS_LOG_UNCOND ("ROUT  " + PrintReceivedPacket (socket, packet));
    }
}

static double
getDistSq(Ptr<Node> n1, Ptr<Node> n2) 
{
  double distSq = 0.0;

  Ptr<MobilityModel> rxPosition = n1->GetObject<MobilityModel> ();
  NS_ASSERT (rxPosition != 0);
  Vector rxPos = rxPosition->GetPosition ();
  double rx_x = rxPos.x;
  double rx_y =rxPos.y;

  Ptr<MobilityModel> txPosition = n2->GetObject<MobilityModel> ();
  NS_ASSERT (txPosition != 0);
  Vector txPos = txPosition->GetPosition ();
  double tx_x = txPos.x;
  double tx_y = txPos.y;
  distSq = (tx_x - rx_x) * (tx_x - rx_x) + (tx_y - rx_y) * (tx_y - rx_y);

  return distSq;
}

void VanetRoutingExperiment::ReceiveWavePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv ()))
    {
      VanetRoutingExperiment::wavePktReceiveCount++;
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      Ptr<Node> node = socket->GetNode ();

      Ptr<MobilityModel> rxPosition = node->GetObject<MobilityModel> ();
      NS_ASSERT (rxPosition != 0);
      Vector rxVel = rxPosition->GetVelocity ();
      // confirm that the receiving node 
      // has also started moving in the scenario
      // if it has not started moving, then
      // it is not a candidate to receive a packet
      int receiverMoving = 1;
      if (rxVel.x != 0.0 || rxVel.y != 0.0)
        {
          receiverMoving = 1;
        }
      else
        {
          receiverMoving = 0;
        }
      if (receiverMoving == 1)
        {
        SocketAddressTag tag;
        bool found;
        found = packet->PeekPacketTag (tag);

        if (found)
          {
            InetSocketAddress addr = InetSocketAddress::ConvertFrom (tag.GetAddress ());
            for (int i = 0; i < (int) m_nNodes; i++)
              {
                if (addr.GetIpv4() == m_adhocTxInterfaces.GetAddress (i) )
                {
                  std::pair<Ptr<Ipv4>, uint32_t> xx = m_adhocTxInterfaces.Get (i);
                  Ptr<Ipv4> pp = xx.first;
                  Ptr<Node> txNode = pp->GetObject<Node> ();

                  double rxDistSq = getDistSq(node, txNode);
                  if (rxDistSq <= VanetRoutingExperiment::m_txSafetyRangeSq)
                    {
                      VanetRoutingExperiment::wavePktInCoverageReceiveCount++;
                    }
                }
              }
          }
        }
    }
}

static void GenerateWaveTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      // first, we make sure this node is moving
      // if not, then skip
      int txNodeId = socket->GetNode ()->GetId ();
      NodeContainer const & n = VanetRoutingExperiment::m_adhocTxNodes;
      Ptr<MobilityModel> txPosition = n.Get (txNodeId) ->GetObject<MobilityModel> ();
      NS_ASSERT (txPosition != 0);

      Vector txVel = txPosition->GetVelocity ();
      int senderMoving = 1;
      if ((txVel.x == 0.0) && (txVel.y == 0.0))
        {
          senderMoving = 0;
        }
      else
        {
          senderMoving = 1;
        }
      if (senderMoving != 0)
        {
        socket->Send (Create<Packet> (pktSize));

        VanetRoutingExperiment::wavePktSendCount++;
        if ((VanetRoutingExperiment::wavePktSendCount % 1000) == 0)
          {
            NS_LOG_UNCOND ("Sending WAVE pkt # " << VanetRoutingExperiment::wavePktSendCount );
          }

        // find other nodes close to this one
        // iterate our nodes and print their position.
        NodeContainer const & n = VanetRoutingExperiment::m_adhocTxNodes;
        for (NodeContainer::Iterator j = n.Begin ();
             j != n.End (); ++j)
          {
            Ptr<Node> object = *j;
            int rxNodeId = object->GetId ();

            if (rxNodeId != txNodeId) 
              {
              Ptr<MobilityModel> rxPosition = object->GetObject<MobilityModel> ();
              NS_ASSERT (rxPosition != 0);
              Vector rxVel = rxPosition->GetVelocity ();
              // confirm that the receiving node 
              // has also started moving in the scenario
              // if it has not started moving, then
              // it is not a candidate to receive a packet
              int receiverMoving = 1;
              if (rxVel.x != 0.0 || rxVel.y != 0.0)
                {
                  receiverMoving = 1;
                }
              else
                {
                  receiverMoving = 0;
                }
              if (receiverMoving == 1)
                {
                double distSq = getDistSq(n.Get (txNodeId), object);
                if (distSq <= VanetRoutingExperiment::m_txSafetyRangeSq)
                  {
                    VanetRoutingExperiment::wavePktExpectedReceiveCount++;
                  }
                }
              }
          }
        }

      Simulator::Schedule (pktInterval, &GenerateWaveTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

void
VanetRoutingExperiment::CheckThroughput ()
{
  double kbs = (bytesTotal * 8.0) / 1000;
  double wavePDR = 0.0;
  if (VanetRoutingExperiment::wavePktSendCount > 0) {
    wavePDR = (double) VanetRoutingExperiment::wavePktReceiveCount / (double) VanetRoutingExperiment::wavePktSendCount;
    }

  double wavePDR2 = 0.0;
  if (VanetRoutingExperiment::wavePktExpectedReceiveCount > 0) {
    wavePDR2 = (double) VanetRoutingExperiment::wavePktInCoverageReceiveCount / (double) VanetRoutingExperiment::wavePktExpectedReceiveCount;
    }

  bytesTotal = 0;
  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  NS_LOG_UNCOND ("CheckThroughput at " << (Simulator::Now ()).GetSeconds () << " Rx=" << VanetRoutingExperiment::wavePktInCoverageReceiveCount << " of Tx=" <<  VanetRoutingExperiment::wavePktExpectedReceiveCount << " PDR=" << wavePDR2);

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << m_nSinks << ","
      << m_protocolName << ","
      << m_txp << ","
      << VanetRoutingExperiment::wavePktReceiveCount << ","
      << VanetRoutingExperiment::wavePktSendCount << ","
      << wavePDR << ","
      << VanetRoutingExperiment::wavePktExpectedReceiveCount << ","
      << VanetRoutingExperiment::wavePktInCoverageReceiveCount << ","
      << wavePDR2 << ""
      << std::endl;

  out.close ();
  packetsReceived = 0;
  VanetRoutingExperiment::wavePktReceiveCount = 0;
  VanetRoutingExperiment::wavePktSendCount = 0;

  Simulator::Schedule (Seconds (1.0), &VanetRoutingExperiment::CheckThroughput, this);
}

Ptr<Socket>
VanetRoutingExperiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&VanetRoutingExperiment::ReceivePacket, this));

  return sink;
}

void
VanetRoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd;
  double txDist = 145.0;
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
  cmd.AddValue ("totaltime", "Simulation end time", m_TotalTime);
  cmd.AddValue ("nodes", "Number of nodes (i.e. vehicles)", m_nNodes);
  cmd.AddValue ("sinks", "Number of routing sinks", m_nSinks);
  cmd.AddValue ("txp", "Transmit power (dB), e.g. txp=7.5", m_txp);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.AddValue ("lossModel", "1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance", m_lossModel);
  cmd.AddValue ("phyMode", "Wifi Phy mode", m_phyMode);
  cmd.AddValue ("80211Mode", "1=802.11p; 2=802.11b", m_80211mode);
  cmd.AddValue ("traceFile", "Ns2 movement trace file", m_traceFile);
  cmd.AddValue ("logFile", "Log file", m_logFile);
  cmd.AddValue ("mobility", "1=trace;2=RWP", m_mobility);
  cmd.AddValue ("rate", "Rate", m_rate);
  cmd.AddValue ("phyMode_b", "Phy mode 802.11b", m_phyMode_b);
  cmd.AddValue ("speed", "Node speed (m/s)", m_nodeSpeed);
  cmd.AddValue ("pause", "Node pause (s)", m_nodePause);
  cmd.AddValue ("verbose", "0=quiet;1=verbose", m_verbose);
  cmd.AddValue ("bsm", "(WAVE) BSM size (bytes)", m_wavePacketSize);
  cmd.AddValue ("interval", "(WAVE) BSM interval (s)", m_waveInterval);
  cmd.AddValue ("scenario", "1=playback(abc)", m_scenario);
  cmd.AddValue ("flowmon", "o=off; 1=on", m_flowmon);
  cmd.AddValue ("txdist", "Expected BSM tx range, m", txDist);
  cmd.AddValue ("gpsaccurcy", "GPS time accuracy, in ns", m_gpsAccuracyNs);
  cmd.AddValue ("buildings", "Load building (obstacles)", m_loadBuildings);
  cmd.Parse (argc, argv);

  VanetRoutingExperiment::m_txSafetyRangeSq = txDist * txDist;
}

void
VanetRoutingExperiment::SetupLogFile () 
{
  // open log file for output
  m_os.open (m_logFile.c_str ());
}

void VanetRoutingExperiment::SetupLogging ()
{
 
  // Enable logging from the ns2 helper
  LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  Packet::EnablePrinting ();
}

void 
VanetRoutingExperiment::ConfigureDefaults ()
{
  Config::SetDefault ("ns3::OnOffApplication::PacketSize",StringValue ("64"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (m_rate));
 
  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyMode_b));

  // Configure callback for logging
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeBoundCallback (&CourseChange, &m_os));
}

void
VanetRoutingExperiment::SetupAdhocMobilityNodes ()
{
  if (m_mobility == 1)
    {
    // Create Ns2MobilityHelper with the specified trace log file as parameter
    Ns2MobilityHelper ns2 = Ns2MobilityHelper (m_traceFile);

    VanetRoutingExperiment::m_adhocTxNodes.Create (m_nNodes);

    ns2.Install (); // configure movements for each node, while reading trace file
    }
  else if (m_mobility == 2) 
    {
    MobilityHelper mobilityAdhoc;
    int64_t streamIndex = 0; // used to get consistent mobility across scenarios

    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    // we need antenna height uniform [1.0 .. 2.0] for loss model
    pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    streamIndex += taPositionAlloc->AssignStreams (streamIndex);

    VanetRoutingExperiment::m_adhocTxNodes.Create (m_nNodes);

    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << m_nodeSpeed << "]";
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << m_nodePause << "]";
    mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                    "Speed", StringValue (ssSpeed.str ()),
                                    "Pause", StringValue (ssPause.str ()),
                                    "PositionAllocator", PointerValue (taPositionAlloc));
    mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
    mobilityAdhoc.Install (VanetRoutingExperiment::m_adhocTxNodes);
    streamIndex += mobilityAdhoc.AssignStreams (VanetRoutingExperiment::m_adhocTxNodes, streamIndex);
    }
 }

void
VanetRoutingExperiment::SetupAdhocDevices ()
{
  if (m_lossModel == 1) 
    {
    m_lossModelName = "ns3::FriisPropagationLossModel";
    }
  else if (m_lossModel == 2) 
    {
    m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
    }
  else if (m_lossModel == 3)
    {
    m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
    }
  else if (m_lossModel == 4) 
    {
    m_lossModelName = "ns3::LogDistancePropagationLossModel";
    }

  // frequency
  double freq = 0.0;
  if (m_80211mode == 1)
    {
      // 802.11p 5.9 GHz
      freq = 5.9e9;
    }
  else 
    {
      // 802.11b 2.4 GHz
      freq = 2.4e9;
    }

  // Setup propagation models 
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (m_verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  WifiHelper wifi;

  // Setup 802.11b stuff
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyMode_b),
                                "ControlMode",StringValue (m_phyMode_b));

  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Set Tx Power
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Setup net devices

  if (m_80211mode == 1) 
    {
    m_adhocTxDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, VanetRoutingExperiment::m_adhocTxNodes);
    }
  else 
    {
    m_adhocTxDevices = wifi.Install (wifiPhy, wifiMac, VanetRoutingExperiment::m_adhocTxNodes);
    }

  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (m_tr_name + ".tr").c_str());
  wifiPhy.EnableAsciiAll (osw);
  wifiPhy.EnablePcapAll("vanet-routing-compare-pcap");
}

void
VanetRoutingExperiment::SetupRouting ()
{

  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  Time rtt = Time(5.0);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> rtw = ascii.CreateFileStream ("routing_table");

  switch (m_protocol)
    {
    case 1:
      olsr.PrintRoutingTableAllAt(rtt, rtw);
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 0:
    case 2:
      aodv.PrintRoutingTableAllAt(rtt, rtw);
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      dsdv.PrintRoutingTableAllAt(rtt, rtw);
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

   if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);
      internet.Install (VanetRoutingExperiment::m_adhocTxNodes);
    }
  else if (m_protocol == 4)
    {
      internet.Install (VanetRoutingExperiment::m_adhocTxNodes);
      dsrMain.Install (dsr, VanetRoutingExperiment::m_adhocTxNodes);
    }
}

void
VanetRoutingExperiment::AssignIpAddresses ()
{
  NS_LOG_INFO ("assigning ip address");
  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.0.0", "255.255.0.0");
  m_adhocTxInterfaces = addressAdhoc.Assign (m_adhocTxDevices);
}

void
VanetRoutingExperiment::SetupWaveMessages ()
{
  // setup generation of WAVE BSM messages
  Time waveInterPacketInterval = Seconds (m_waveInterval);

  // arbitrary
  int wavePort = 9080;

  double startTime = 1.0;
  // total length of time transmitting WAVE packets
  double totalTxTime = m_TotalTime - startTime;
  // total WAVE packets needing to be sent
  m_numWavePackets = (uint32_t) (totalTxTime / m_waveInterval);

  // first node received WAVE BSM from all others
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // all nodes except first send WAVE BSM
  for (int i = 0; i < (int) m_nNodes; i++)
    {
    Ptr<Socket> recvSink = Socket::CreateSocket (VanetRoutingExperiment::m_adhocTxNodes.Get (i), tid);
    recvSink->SetRecvCallback (MakeCallback (&VanetRoutingExperiment::ReceiveWavePacket, this));
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), wavePort);
    recvSink->Bind (local);
    recvSink->BindToNetDevice (VanetRoutingExperiment::m_adhocTxDevices.Get (i));
    recvSink->SetAllowBroadcast (true);

    InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), wavePort);
    recvSink->Connect (remote);

    UniformVariable n(1, m_gpsAccuracyNs);   
    int t=n.GetValue();       

    Time time = Seconds(startTime + (double) t / 1000000.0);

    Simulator::ScheduleWithContext (recvSink->GetNode ()->GetId (),
                                    time, &GenerateWaveTraffic,
                                    recvSink, m_wavePacketSize, m_numWavePackets, waveInterPacketInterval);
    }
}

void
VanetRoutingExperiment::SetupRoutingMessages ()
{
  // Setup routing transmissions
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  for (int i = 0; i <= m_nSinks - 1; i++)
    {
      // protocol == 0 means no routing data, WAVE BSM only
      // so do not set up sink
      if (m_protocol != 0)
        {
        Ptr<Socket> sink = SetupPacketReceive (m_adhocTxInterfaces.GetAddress (i), VanetRoutingExperiment::m_adhocTxNodes.Get (i));
        }

      AddressValue remoteAddress (InetSocketAddress (m_adhocTxInterfaces.GetAddress (i), port));
      onoff1.SetAttribute ("Remote", remoteAddress);

      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (VanetRoutingExperiment::m_adhocTxNodes.Get (i + m_nSinks));
      temp.Start (Seconds (var->GetValue (1.0,2.0)));
      temp.Stop (Seconds (m_TotalTime));
    }
}

void
VanetRoutingExperiment::SetupScenario()
{
  // member variable parameter use
  // defaults or command line overrides,
  // except where scenario={1,2,3,...} 
  // have been specified, in which case
  // specify parameters are overwritten
  // here to setup for specific scenarios

  // certain parameters may be further overridden
  // i.e. specify a scenario, yet override tx power.

  if (m_scenario == 1) 
    {
    // 40 nodes in RWP 300 m x 1500 m synthetic highway, 10s
    m_traceFile = "";
    m_logFile = "";
    m_mobility = 2;
    if (m_nNodes == 156)
      {
        m_nNodes = 40;
      }
    if (m_TotalTime == 300.01)
      {
        m_TotalTime = 10.0;
      }
    if (m_loadBuildings != 0) 
      {
        std::string bldgFile = "scratch/highway.buildings.xml";
        Topology::LoadBuildings(bldgFile);
      }
    }
  else if (m_scenario == 2) 
    {
    // Realistic vehicular trace in 4.6 km x 3.0 km suburban Zurich
    // "low density"
    m_traceFile = "./scratch/low_ct-unterstrass-1day.filt.5.adj.mov";
    m_logFile = "low_ct-unterstrass-1day.filt.5.adj.log";
    m_mobility = 1;
    m_nNodes = 156;
    m_TotalTime = 300.01;
    m_nodeSpeed = 0;
    m_nodePause = 0;
    m_CSVfileName = "low_vanet-routing-compare.csv";
    }
  else if (m_scenario == 3) 
    {
    // Realistic vehicular trace in 4.6 km x 3.0 km suburban Zurich
    // "med density"
    m_traceFile = "./scratch/low_ct-unterstrass-1day.filt.5.adj.mov";
    m_logFile = "low_ct-unterstrass-1day.filt.5.adj.log";
    m_mobility = 1;
    m_nNodes = 156;
    m_TotalTime = 300.01;
    m_nodeSpeed = 0;
    m_nodePause = 0;
    m_CSVfileName = "low_vanet-routing-compare.csv";
    }
  else if (m_scenario == 4) 
    {
    // Realistic vehicular trace in 4.6 km x 3.0 km suburban Zurich
    // "high density"
    m_traceFile = "./scratch/low_ct-unterstrass-1day.filt.5.adj.mov";
    m_logFile = "low_ct-unterstrass-1day.filt.5.adj.log";
    m_mobility = 1;
    m_nNodes = 156;
    m_TotalTime = 300.01;
    m_nodeSpeed = 0;
    m_nodePause = 0;
    m_CSVfileName = "low_vanet-routing-compare.csv";
    }
  else if (m_scenario == 5) 
    {
    // Centennial campus
    m_traceFile = "./scratch/centennial2.ns2";
    m_logFile = "centennial2.log";
    m_mobility = 1;
    m_nNodes = 180;
    m_TotalTime = 781;
    m_nodeSpeed = 0;
    m_nodePause = 0;
    m_CSVfileName = "centennial2.csv";
    // WAVE BSM only
    m_protocol = 0;
    m_txSafetyRangeSq = 145.0 * 145.0;
    if (m_loadBuildings != 0) 
      {
        std::string bldgFile = "scratch/centennial1.buildings.xml";
        Topology::LoadBuildings(bldgFile);
      }
    }
  if (m_txp == 7.5)
    {
      m_txp = 20;
    }
}

void
VanetRoutingExperiment::Run ()
{
  WriteCsvHeader();
  SetupScenario();
  SetupLogFile ();
  SetupLogging ();
  ConfigureDefaults ();
  int nWifis = m_nNodes;
  SetupAdhocMobilityNodes ();
  SetupAdhocDevices();
  SetupRouting ();
  AssignIpAddresses ();
  SetupWaveMessages ();
  SetupRoutingMessages ();

  AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (m_tr_name + ".mob"));

  // Enable flowmon capture
  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  if (m_flowmon != 0)
    {
    flowmon = flowmonHelper.InstallAll ();
    }

  NS_LOG_INFO ("Run Simulation.");

  CheckThroughput ();

  Simulator::Stop (Seconds (m_TotalTime));
  Simulator::Run ();

  if (m_flowmon != 0)
    {
    flowmon->SerializeToXmlFile ((m_tr_name + ".flowmon").c_str(), false, false);
    }

  Simulator::Destroy ();

  m_os.close (); // close log file
}

void
VanetRoutingExperiment::WriteCsvHeader ()
{
  //blank out the last output file and write the column headers
  std::ofstream out (m_CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  "RoutingProtocol," <<
  "TransmissionPower," <<
  "WavePktsSent," <<
  "WavePtksReceived," <<
  "WavePktsPpr," <<
  "ExpectedWavePktsReceived," <<
  "ExpectedWavePktsInCoverageReceived," <<
  "BSM_PDR" <<
  std::endl;
  out.close ();
}

int
main (int argc, char *argv[])
{
  VanetRoutingExperiment experiment;
  experiment.CommandSetup (argc,argv);

  experiment.Run ();
}
