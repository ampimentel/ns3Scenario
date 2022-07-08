/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 North Carolina State University
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Scott E. Carpenter <scarpen@ncsu.edu>
 *
 */


/*
  Simpler version of the programme in src/wave/examples/vanet-compare.cc
  We want to generate application traffic with WAVE BSM and  routing traffic with AOVD
  We will simulate 50 nodes in random waypoint mobilit model and will try to collect metrics


*/
#include <atomic> 
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/v4ping-helper.h"
#include "ns3/address.h"

using namespace ns3;



NS_LOG_COMPONENT_DEFINE ("vanet-routing-compare");

class SendOnRecv : public Application 
{
public:

  SendOnRecv ();
  virtual ~SendOnRecv();

  void Setup (int port, uint32_t packetSize, uint32_t nPackets, DataRate dataRate, Time procTime);
  void SetTxCallback (Callback<void, const Address &> receivedData);
  static bool isBaseStationClient(Ipv4Address ip, uint16_t port);

  static uint16_t destinationPort;
  static std::map<InetSocketAddress, bool>  baseStationsClients;

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (Address sendAddr, int &numPackets);
  void SendPacket (Address sendAddr, int &numPackets);
  void HandleRead (Ptr<Socket> socket);

  Ptr<Socket>     m_socket;
  int             m_port;
  Time            m_processing_time;
  uint32_t        m_packetSize;
  uint32_t        m_maxPackets;
  std::map<Address, int>        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
  Callback<void, const Address &> m_txTraceWithAddresses;
  float           m_delay_milis;
};

uint16_t SendOnRecv::destinationPort = 0;
std::map<InetSocketAddress, bool> SendOnRecv::baseStationsClients;

bool SendOnRecv::isBaseStationClient(Ipv4Address ip, uint16_t port){
  InetSocketAddress toSearch(ip, port);
  return (baseStationsClients.count(toSearch) > 0);
}


void
SendOnRecv::SetTxCallback (Callback<void, const Address &> receivedData)
{
  m_txTraceWithAddresses = receivedData;
}

SendOnRecv::SendOnRecv ()
  : m_socket (0),
    m_processing_time(MilliSeconds(0.0)), 
    m_packetSize (0), 
    m_maxPackets(0), 
    m_dataRate (0), 
    m_sendEvent (), 
    m_running (false),
    m_packetsSent (0)
{
}

SendOnRecv::~SendOnRecv()
{
  m_socket = 0;
}

void
SendOnRecv::Setup (int port, uint32_t packetSize, uint32_t nPackets, DataRate dataRate, Time proc)
{
  m_packetSize = packetSize;
  m_maxPackets = nPackets;
  m_dataRate = dataRate;
  m_port = port;
  m_processing_time = proc;
}

void
SendOnRecv::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  m_socket = Socket::CreateSocket (GetNode (), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), m_port);
  if (m_socket->Bind (local) != -1)
  m_socket->Listen();
  m_socket->SetRecvCallback (MakeCallback (&SendOnRecv::HandleRead, this));
}

void 
SendOnRecv::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
 
  Ptr<Packet> packet_aux;
  Address from;
  Address localAddress;
  while ((packet_aux = socket->RecvFrom (from)))
    {

      if(m_nPackets.count(from) == 0){
        m_nPackets.insert({from, m_maxPackets - 1});
        destinationPort = InetSocketAddress::ConvertFrom(from).GetPort();
        baseStationsClients.insert(std::pair<InetSocketAddress, bool>(InetSocketAddress::ConvertFrom(from), true));
        Simulator::Schedule (m_processing_time, &SendOnRecv::SendPacket, this, from,  m_nPackets[from]);
      }
    }
}

void 
SendOnRecv::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void 
SendOnRecv::SendPacket (Address sendAddr, int &numPackets)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->SendTo (packet, 0, sendAddr);
  m_txTraceWithAddresses (sendAddr);
  //Adds Processing Time

  if (numPackets > 0)
    {
      ScheduleTx (sendAddr, --numPackets);
    }
}

void 
SendOnRecv::ScheduleTx (Address sendAddr, int &numPackets)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &SendOnRecv::SendPacket, this, sendAddr, numPackets);
    }
}

std::atomic<int> numSend(0);
static void tracePacketSent(const Address  & out) {
  numSend++;
}

std::atomic<int> numRecv(0);
static void tracePacketRecieveBs(std::string context, Ptr< const Packet > packet) {
  numRecv++;
}

std::atomic<int> txSend(0), rxReceive(0), echoServerReceive(0);
int tx2Send = 0; 
void  TxPacketReceived(std::string context, Ptr< const Packet > packet) {
  txSend++;
  return;
}

void  RxPacketReceived(std::string context, Ptr< const Packet > packet) {
  rxReceive++;
  return;
}

void  echoServerPacketReceived(std::string context, Ptr< const Packet > packet) {
  echoServerReceive++;
  return;
}

void
PrintWaveStats (WaveBsmHelper m_waveBsmHelper)
{
  double wavePDR = 0.0;
  int wavePktsSent = m_waveBsmHelper.GetWaveBsmStats ()->GetTxPktCount ();
  int wavePktsReceived = m_waveBsmHelper.GetWaveBsmStats ()->GetRxPktCount ();
  if (wavePktsSent > 0)
    {
      int wavePktsReceived = m_waveBsmHelper.GetWaveBsmStats ()->GetRxPktCount ();
      wavePDR = (double) wavePktsReceived / (double) wavePktsSent;
    }

  int waveExpectedRxPktCount = m_waveBsmHelper.GetWaveBsmStats ()->GetExpectedRxPktCount (1);
  int waveRxPktInRangeCount = m_waveBsmHelper.GetWaveBsmStats ()->GetRxPktInRangeCount (1);
  double wavePDR1_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (1);
  double wavePDR2_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (2);
  double wavePDR3_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (3);
  double wavePDR4_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (4);
  double wavePDR5_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (5);
  double wavePDR6_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (6);
  double wavePDR7_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (7);
  double wavePDR8_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (8);
  double wavePDR9_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (9);
  double wavePDR10_2 = m_waveBsmHelper.GetWaveBsmStats ()->GetBsmPdr (10);

  // calculate MAC/PHY overhead (mac-phy-oh)
  // total WAVE BSM bytes sent
  // mac-phy-oh = (total-phy-bytes - total-app-bytes) / total-phy-bytes
  std::cout << "Time, PktsSent, PktsReceived, PDR, ExpectedPktsReceived, RxPktInRange, ..." << std::endl;
  std::cout << (Simulator::Now ()).As (Time::S) << ","
      << wavePktsSent << ","
      << wavePktsReceived << ","
      << wavePDR << ","
      << waveExpectedRxPktCount << ","
      << waveRxPktInRangeCount << ","
      << wavePDR1_2 << ","
      << wavePDR2_2 << ","
      << wavePDR3_2 << ","
      << wavePDR4_2 << ","
      << wavePDR5_2 << ","
      << wavePDR6_2 << ","
      << wavePDR7_2 << ","
      << wavePDR8_2 << ","
      << wavePDR9_2 << ","
      << wavePDR10_2 << ","
      << std::endl;
}

void generate_node_udp_traffic(NodeContainer node, int num_nodes, int end, int beg, int avgDataRate, int maxConn, Ptr<UniformRandomVariable> rand){
    
    //int rand_numConnections = rand->GetInteger(0, maxConn);
    int PacketSize = 1024;
    uint32_t node_id = node.Get(0)->GetId();
    for (int j = 0; j < 1; j++) {
      int startTime =  beg;//rand->GetInteger(1, beg);
      int endTime = end;//rand->GetInteger(startTime + 1, end);
      int maxPacket = 2 * avgDataRate * (endTime - startTime) / (PacketSize * 8) + 1;
      int nPackets =  rand->GetInteger(1, maxPacket);
      //std::cout << "MAXPACKET: " << maxPacket << " nPackets: " <<  nPackets << std::endl;
      float interval = (float) (endTime - startTime) / (nPackets+1);
      uint32_t toSend = rand->GetInteger(1, num_nodes); //Ip from 1 to num_nodes + 1
      toSend = toSend == node_id + 1 ? (toSend + 1) % (num_nodes+1) : toSend; //cannot send to itself
      toSend = toSend == 0 ? toSend + 1 : toSend; //cannot send to 0 ip
      std::stringstream addrStr;
      addrStr << "10.1.1." << toSend;
      Ipv4Address addrIp(addrStr.str().c_str());
      //std::cout << "sendto: " << addrIp << std::endl;
      UdpEchoClientHelper echoClient (addrIp, 9);
      echoClient.SetAttribute ("MaxPackets", UintegerValue (nPackets));
      echoClient.SetAttribute ("Interval", TimeValue (Seconds (interval)));
      echoClient.SetAttribute ("PacketSize", UintegerValue (PacketSize));
      ApplicationContainer clientApps = echoClient.Install (node);
      clientApps.Start (Seconds (startTime));
      clientApps.Stop (Seconds (endTime+1));
      std::ostringstream oss1, oss2;
      oss1 << "/NodeList/" << node_id << "/ApplicationList/"<< node.Get(0)->GetNApplications() - 1 <<"/$ns3::UdpEchoClient/Tx";
      Config::Connect(oss1.str(),  MakeCallback(&TxPacketReceived));
      oss2 << "/NodeList/" << node_id << "/ApplicationList/"<< node.Get(0)->GetNApplications() - 1 <<"/$ns3::UdpEchoClient/Rx";
      Config::Connect(oss2.str(),  MakeCallback(&RxPacketReceived));
      tx2Send += nPackets;
    }

    return;
  }

bool
Receive3 (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  std::cout << "receive a packet: " << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().As (Time::S) << "," << std::endl
            << "  recvTime = " << Now ().As (Time::S) << "," << std::endl
            << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;
  return true;
}

void setupMobility (int mobTrace, std::string mobFile, NodeContainer nodes, NodeContainer baseStations) {

  MobilityHelper mobilityBaseStation;
  ObjectFactory pos;
  pos.SetTypeId ("ns3::ListPositionAllocator");
  Ptr<ListPositionAllocator> basePa = pos.Create ()->GetObject<ListPositionAllocator> ();

  if(mobTrace >= 1 && mobTrace <= 4)
  {
    std::stringstream fileName; 
    fileName << mobFile << mobTrace << ".tcl";
    Ns2MobilityHelper ns2 = Ns2MobilityHelper (fileName.str());
    ns2.Install ();
    //read base station coordinates from file
    std::string myText;
    std::ifstream bsFile(mobFile + ".bs");
    while (getline (bsFile, myText)) {
        float x, y, z;
        sscanf(myText.c_str(), "%f,%f,%f", &x, &y, &z);
        std::cout << x << " " << y << " " << z << std::endl;
        basePa->Add(Vector(x, y, z));
    }
  }

  else {
    //Mobility
    MobilityHelper mobility;
    //Generates initial positions acording to a seed
    uint64_t mobility_seed = 0;
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
    pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=750.0]"));
    pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=150.0]"));
    //we need antenna height uniform [1.0 .. 2.0] for loss model
    pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));
    Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    mobility_seed += taPositionAlloc->AssignStreams (mobility_seed);
    //Sets mobility model
    int node_speed = 10, node_pause = 1;
    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << node_speed << "]";
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << node_pause << "]";
    mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                "Speed", StringValue (ssSpeed.str ()),
                                "Pause", StringValue (ssPause.str ()),
                                "PositionAllocator", PointerValue (taPositionAlloc));
    //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    
    mobility.SetPositionAllocator (taPositionAlloc);
    mobility.Install (nodes);
    mobility_seed += mobility.AssignStreams (nodes, mobility_seed);
    //base station position
    basePa->Add(Vector(750, 150, 0));
    basePa->Add(Vector(150, 150, 0));
    basePa->Add(Vector(1000, 300, 0));  
  }

  mobilityBaseStation.SetPositionAllocator(basePa);
  mobilityBaseStation.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityBaseStation.Install (baseStations);
}

void generateTraffic(WaveBsmHelper m_waveBsmHelper, NodeContainer nodes, float end, float start, int dataRate, std::vector <double> &m_txSafetyRangesSq) {
  uint32_t num_nodes = nodes.GetN();
  //Generate UDP traffic
  UdpEchoServerHelper echoServer(9);
  ApplicationContainer serverApps = echoServer.Install(nodes);
  serverApps.Start (Seconds (start));
  serverApps.Stop (Seconds (end*1.2));
  for(uint32_t i = 0; i < num_nodes; i++){
    Ptr<Node> nd = nodes.Get(i);
    std::ostringstream oss;
    oss << "/NodeList/" << nd->GetId() << "/ApplicationList/"<< nd->GetNApplications() - 1 <<"/$ns3::UdpEchoServer/Rx";
    Config::Connect(oss.str(),  MakeCallback(&echoServerPacketReceived));
  }
  
  Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable> ();
  for(uint32_t i = 0; i < num_nodes; i++) {
    generate_node_udp_traffic(nodes.Get(i), num_nodes, end, start, dataRate, 5, rand);
  }

  WaveBsmHelper::GetNodesMoving ().resize (num_nodes, 1);
  m_txSafetyRangesSq.resize (10, 0);
  m_txSafetyRangesSq[0] = 50.0 * 50.0;
  m_txSafetyRangesSq[1] = 100.0 * 100.0;
  m_txSafetyRangesSq[2] = 200.0 * 200.0;
  m_txSafetyRangesSq[3] = 300.0 * 300.0;
  m_txSafetyRangesSq[4] = 400.0 * 400.0;
  m_txSafetyRangesSq[5] = 500.0 * 500.0;
  m_txSafetyRangesSq[6] = 600.0 * 600.0;
  m_txSafetyRangesSq[7] = 800.0 * 800.0;
  m_txSafetyRangesSq[8] = 1000.0 * 1000.0;
  m_txSafetyRangesSq[9] = 1500.0 * 1500.0;
}

void configureApplicationsToTest(uint8_t baseStationPort, float end, float start, NodeContainer baseStations, NodeContainer nodes, Time processingTime, int nPackets) {
  
  //Base Station Apps => Send
  UdpEchoClientHelper baseStationClient(Ipv4Address("10.1.1.255"), baseStationPort);
  baseStationClient.SetAttribute ("MaxPackets", UintegerValue (10*(end-start)));
  baseStationClient.SetAttribute ("Interval", TimeValue (Seconds(0.1)));
  baseStationClient.SetAttribute ("PacketSize", UintegerValue (1024));
  ApplicationContainer apps = baseStationClient.Install (baseStations);
  apps.Start (Seconds (start));
  apps.Stop (Seconds (end));

  for(uint32_t i = 0; i < baseStations.GetN(); i++) {
    std::ostringstream oss;
    oss << "/NodeList/" << baseStations.Get(i)->GetId() << "/ApplicationList/" << baseStations.Get(i)->GetNApplications()-1 << "$ns3::UdpEchoClient/Rx";
    Config::Connect(oss.str(),  MakeCallback(&tracePacketRecieveBs));
  }
  
  uint32_t num_nodes = nodes.GetN();
  for(uint32_t i = 0; i < num_nodes; i++) {
    Ptr<SendOnRecv> appRecv = CreateObject<SendOnRecv> ();
    appRecv->Setup (baseStationPort, 1024, nPackets, DataRate ("1Mbps"), processingTime);
    appRecv->SetTxCallback(MakeCallback(&tracePacketSent));
    nodes.Get (i)->AddApplication (appRecv);
    appRecv->SetStartTime (Seconds (start));
    appRecv->SetStopTime (Seconds (end));  
  }
}

void printSimStats(WaveBsmHelper m_waveBsmHelper, Ptr<FlowMonitor> flowMonitor, FlowMonitorHelper &flowHelper, Ipv4InterfaceContainer baseStationInterface) {
  
  PrintWaveStats(m_waveBsmHelper);
  std::cout << "Echo Client Packets Sent: " << txSend << "\n";
  std::cout << "Echo Server Packets Received: " << echoServerReceive << "\n";
  std::cout << "Echo Client Packets Received: " << rxReceive << "\n";
  std::cout << "Packets Sent to Base Station " << numSend << "\n";
  std::cout << "Packets Recieve in the Base Station " << numRecv << "\n";
  
  Time total_delay(0);
  int num_flows = 0, total_lost = 0, total_sent = 0, total_rx = 0;
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();
  for(auto const& flow : stats) {
    Ipv4FlowClassifier::FiveTuple flowInfo = classifier->FindFlow(flow.first);
    if (SendOnRecv::isBaseStationClient(flowInfo.destinationAddress, flowInfo.destinationPort)) {
      if(flow.second.rxPackets > 0) {
        total_delay += flow.second.delaySum / flow.second.rxPackets;
        num_flows++;
      }
      total_sent += flow.second.txPackets;
      total_rx += flow.second.rxPackets;
      total_lost += flow.second.lostPackets;
    }
  }
  
  std::cout << "Average Delay (ms): " << total_delay.GetMilliSeconds() / (num_flows == 0 ? 1 : num_flows) << std::endl;
  std::cout << "\tNum Flows: " << num_flows  << std::endl;
  std::cout << "\tTotal Lost Packets: " << total_lost  << std::endl;
  std::cout << "\tTotal Sent Packets: " << total_sent  << std::endl;
  std::cout << "\tTotal Received Packets: " << total_rx  << std::endl;
}


int main (int argc, char *argv[])
{
  std::cout << std::endl << "===BEGIN SIMULATION===" << std::endl;

  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  float start = 1.0, end = 201;
  int dataRate = 3e3;
  int num_nodes = 50;
  int num_baseStations = 1;
  int mobTrace = 1;
  int scenario = 1;
  float processingTime = 30.0;
  //int numKeywords = 1;
  int nPackets = 1;
  //int numDisjunctions = 1;

  CommandLine cmd (__FILE__);

  cmd.AddValue("start", "start time for simulation", start);
  cmd.AddValue("end", "end time for simulation", end);
  cmd.AddValue("dataRate", "average data rate for vehicles in the network", dataRate);
  cmd.AddValue ("numNodes", "number of nodes in simmulation", num_nodes);
  cmd.AddValue ("numBaseStations", "number of baseStations in simmulation", num_baseStations);
  cmd.AddValue ("mobTrace", "number of the mobility trace to use 1-3, dont use mobility trace 0", mobTrace);
  cmd.AddValue ("scenario", "scenario to simulate", scenario);
  cmd.AddValue ("nPackets", "number of packets to send", nPackets);
  //cmd.AddValue ("numDisjunctions", "number of disjunctions on query", scenario);
  cmd.AddValue ("procTime", "processing time, in miliseconds, of the client before sending a packet", processingTime);

  cmd.Parse (argc, argv);

  NodeContainer nodes, baseStation;
  nodes.Create(num_nodes);
  baseStation.Create(num_baseStations);

  //Setup wifi channel normally and Physical layer for wave
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (channel.Create());

  wavePhy.Set("TxPowerStart", StringValue("23.0"));
  wavePhy.Set("TxPowerEnd", StringValue("23.0"));
  wavePhy.Set("TxGain", StringValue("15.0"));
  wavePhy.Set("RxGain", StringValue("5.0"));
  wavePhy.Set("Frequency", UintegerValue(5900));
  wavePhy.Set("ChannelWidth", UintegerValue(10));
  //Setup mac layer
  Wifi80211pHelper waveHelper = Wifi80211pHelper::Default ();
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();

  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  
  NetDeviceContainer devices = waveHelper.Install (wavePhy, waveMac, nodes);
  NetDeviceContainer baseStationDevice =  waveHelper.Install (wavePhy, waveMac, baseStation);
    

  setupMobility(mobTrace, "examples/NS3Scenario/mobility", nodes, baseStation);
  
  //Network Layer
  InternetStackHelper internet;
  AodvHelper aodv;
  internet.SetRoutingHelper(aodv);
  internet.Install(nodes);
  internet.Install(baseStation);
  //AsciiTraceHelper ascii;
  //aodv.PrintRoutingTableAllAt(Time(9.0),  ascii.CreateFileStream ("routing_table"));

  Ipv4AddressHelper ipv4;
  //NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);
  Ipv4InterfaceContainer baseStationInterface = ipv4.Assign(baseStationDevice);

  //generate realistic traffic
  WaveBsmHelper m_waveBsmHelper;
  std::vector <double> m_txSafetyRangesSq;
  generateTraffic(m_waveBsmHelper, nodes, end, start, dataRate, m_txSafetyRangesSq);
  m_waveBsmHelper.Install (interfaces, Seconds (end-start), 200, Seconds (0.1), 40, m_txSafetyRangesSq, 1, MilliSeconds (10));

  //configure application to test
  uint8_t baseStationPort = 47;
  //scenario 1 => cars pass through baseStations and send encryption information to it
  if(scenario == 1){
    //processingTime = 12 * numKeywords + 14;
    //int nPackets = ceil(50*(0.354*numKeywords + 1.054) / 65.0); //100 encryption of the scheme, divided by packet capacity
    configureApplicationsToTest(baseStationPort, end, start, baseStation, nodes, MilliSeconds(processingTime), nPackets);
  }
  //scenario 2 => cars pass through baseSations, query dB with trapdoor and get a response
  //if(scenario == 2){
  // processingTime = 12 * numKeywords + 14; 
  //  configureApplicationsToTest(baseStationPort, end, start, baseStation, nodes, MilliSeconds(processingTime));
  //}
  //collect data
  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.InstallAll();

  Simulator::Stop (Seconds (end*1.5));
  Simulator::Run ();
  //print results
  printSimStats(m_waveBsmHelper, flowMonitor, flowHelper, baseStationInterface);

  Simulator::Destroy ();
}


/*
Protocolo de encaminhamento geografico 

*/

/*
Reduzir numero de aplicações => 1
Minimo 50%
4km x 4km => 

Começar biblioteca de encriptação
*/