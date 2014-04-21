/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * This is an example script for AODV manet routing protocol. 
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 */

#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h" 
#include "ns3/v4ping-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/csma-module.h"
#include <iostream>
#include <cmath>


 int numberofBytesRecievedUDP = 0;
 using namespace ns3;

/**
 * \brief Test script.
 * 
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 * 
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 * 
 * ping 10.0.0.4
 */


 class AodvExample 
 {
 public:
 	AodvExample ();
  /// Configure script parameters, \return true on successful configuration
 	bool Configure (int argc, char **argv);
  /// Run simulation
 	void Run ();
  /// Report results
 	void Report (std::ostream & os);

 private:
  ///\name parameters
  //\{
  /// Number of nodes
 	uint32_t size;
  /// Distance between nodes, meters
 	double step;
  /// Simulation time, seconds
 	double totalTime;
  /// Write per-device PCAP traces if true
 	bool pcap;
  /// Print routes if true
 	bool printRoutes;
  //\}

  ///\name network
  //\{
 	NodeContainer nodes;
 	NetDeviceContainer devices;
 	Ipv4InterfaceContainer interfaces;
  //\}

 private:
 	void CreateNodes ();
 	void CreateDevices ();
 	void InstallInternetStack ();
 	void InstallApplications ();
  void InstallUDP ();
  void InstallTCP ();
};

int main (int argc, char **argv)
{
  AodvExample test;
  if (!test.Configure (argc, argv))
   NS_FATAL_ERROR ("Configuration failed. Aborted.");

 test.Run ();
 test.Report (std::cout);
 return 0;
}

//-----------------------------------------------------------------------------
AodvExample::AodvExample () :
size (3),
step (100),
totalTime (40),
pcap (true),
printRoutes (true)
{
}

bool
AodvExample::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
  // LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

  SeedManager::SetSeed (12345);
  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);

  cmd.Parse (argc, argv);
  return true;
}

void
AodvExample::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));

  Simulator::Run ();

  Simulator::Destroy ();
}

void
AodvExample::Report (std::ostream &)
{ 
}

void
AodvExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
  {
   std::ostringstream os;
   os << "node-" << i;
   Names::Add (os.str (), nodes.Get (i));
 }
  // Create static grid
 MobilityHelper mobility;
 mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
   "MinX", DoubleValue (0.0),
   "MinY", DoubleValue (0.0),
   "DeltaX", DoubleValue (step),
   "DeltaY", DoubleValue (0),
   "GridWidth", UintegerValue (size),
   "LayoutType", StringValue ("RowFirst"));
 mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
 mobility.Install (nodes);
  // Ptr<Node> node = nodes.Get(3);
  // Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  // mob->SetPosition(Vector(1e5, 1e5, 1e5)); 
  // Simulator::Schedule (Seconds (totalTime/3), &MobilityModel::SetPosition, mob, Vector (1e5, 1e5, 1e5));
}

void
AodvExample::CreateDevices ()
{
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, nodes); 

  if (pcap)
  {
   wifiPhy.EnablePcapAll (std::string ("aodv"));
 }
}

void
AodvExample::InstallInternetStack ()
{
  AodvHelper aodv;
  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  if (printRoutes)
  {
  	Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
  	aodv.PrintRoutingTableAllAt (Seconds (36), routingStream); 

    // print routing table after 1 sec before deleteing nodes.      
  	Ptr<OutputStreamWrapper> routingStream2 = Create<OutputStreamWrapper> ("aodv2.routes", std::ios::out);
  	aodv.PrintRoutingTableAllAt (Seconds (2), routingStream2);

  }
}

void 
callBackMethodForUDP(Ptr<Socket> socket)
{
  std::cout<< "Node 3 has receiver a packet at time -> "<< (Simulator::Now()).GetSeconds() << std::endl;
}

void
AodvExample::InstallUDP ()
{
  std::cout <<"Installing UDP clients and Servers." << std::endl;
  uint16_t port = 8000; 
  UdpServerHelper server (port);


  // create a UDP server.
  ApplicationContainer apps;
  apps = server.Install (nodes.Get(2));

  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (10.0));

  //
  // Create one UdpClient application to send UDP datagrams from node zero to
  // node one.
  //
  uint32_t MaxPacketSize = 1024;
  Time interPacketInterval = Seconds (2);
  uint32_t maxPacketCount = 6;
  UdpClientHelper client (interfaces.GetAddress (2), port);

  client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));

  apps = client.Install(nodes.Get(0));

  apps.Start (Seconds (2.0));
  apps.Stop (Seconds (10.0));


  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (nodes.Get (2), tid);
  InetSocketAddress local (Ipv4Address::GetAny (), port);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&callBackMethodForUDP));

}
void ReceivePacket (Ptr<const Packet> p, const Address &ad){
  std::cout << "here in receve packet "<< p -> GetSize()<< " " <<(Simulator::Now()).GetSeconds() << std::endl;
}

void
AodvExample::InstallTCP ()
{
  uint16_t servPort = 8001;

  // Create a packet sink to receive these packets on n2...
  PacketSinkHelper sink ("ns3::TcpSocketFactory",
   InetSocketAddress (Ipv4Address::GetAny (), servPort));

  ApplicationContainer apps = sink.Install (nodes.Get(2));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (40.0));

// Create the OnOff applications to send TCP to the server
  OnOffHelper clientHelper ("ns3::TcpSocketFactory", interfaces.GetAddress (2));

  uint32_t MaxPacketSize = 2024;

  clientHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  clientHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  clientHelper.SetAttribute ("PacketSize", UintegerValue (512));
  clientHelper.SetAttribute ("MaxBytes", UintegerValue (MaxPacketSize));

  ApplicationContainer clientApp;
  AddressValue remoteAddress
  (InetSocketAddress (interfaces.GetAddress(2), servPort));
  clientHelper.SetAttribute ("Remote", remoteAddress);
  clientApp.Add (clientHelper.Install(nodes.Get(0)));

  clientApp.Start (Seconds (2.0));
  clientApp.Stop (Seconds (40.0));

  // install the callback.
  //-----------------------------------------------------------------------------------------
  Ptr<Node> node = nodes.Get (2);
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  // Trace Received Packets
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&ReceivePacket));
}

void
AodvExample::InstallApplications ()
{
	// V4PingHelper ping (interfaces.GetAddress (1));
	// ping.SetAttribute ("Verbose", BooleanValue (true));

	// ApplicationContainer p = ping.Install (nodes.Get (1));
	// p.Start (Seconds (0));
	// p.Stop (Seconds (totalTime) - Seconds (0.001));

 //  // move node away
	// Ptr<Node> node = nodes.Get (0);
	// Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
	// Simulator::Schedule (Seconds (0), &MobilityModel::SetPosition, mob, Vector (1e5, 1e5, 1e5));


  // InstallUDP(); 
  InstallTCP();
}

