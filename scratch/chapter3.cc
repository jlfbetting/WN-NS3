/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-model.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/buildings-helper.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/propagation-loss-model.h"

#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace ns3;
using std::vector;
NS_LOG_COMPONENT_DEFINE ("NS3wifiAssignment");
int main (int argc, char *argv[])
{
	int frequency = 2400;  // frequency [MHz]
	double distance = 1;
	bool printOnScreen = true;
	double txPwr = 100;
	std::string outputName = "chapter3OutFile.out";
	CommandLine cmd;

	cmd.AddValue("frequency", "Frequency of the signal [MHz]", frequency);
	cmd.AddValue("printOnScreen", "Only print on screen (true or false)", printOnScreen);
	cmd.AddValue("outputName", "Filename of output file", outputName);
	cmd.AddValue("distance", "The distance between the transmitter and the receiver", distance);
	cmd.AddValue("txPwr", "The power of the transmitter [dBm]", txPwr);
	ConfigStore inputConfig;
	inputConfig.ConfigureDefaults ();
	cmd.Parse(argc, argv);
	// create and open the output file
	std::ofstream outFile;
	outFile.open(outputName.c_str(), std::ofstream::out | std::ofstream::app);
	if (!outFile.is_open ())
	{
		NS_FATAL_ERROR ("Cannot open the output file");
	}

	Ptr<HybridBuildingsPropagationLossModel> hybridBuildingsPropagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();
	hybridBuildingsPropagationLossModel->SetFrequency(frequency *pow(10,6));
	// creating nodes, setting their positions
	NodeContainer apNode;
	NodeContainer staNode;
	apNode.Create(1);
	staNode.Create(1);
	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(apNode);
	mobility.Install(staNode);
	Ptr<MobilityModel> mobilityAp;
	Ptr<MobilityModel> mobilitySta;
	Vector posA;
	Vector posS;
	mobilityAp = apNode.Get(0)->GetObject<MobilityModel> ();
	mobilitySta = staNode.Get(0)->GetObject<MobilityModel> ();
	posA = mobilityAp->GetPosition();
	posS = mobilitySta->GetPosition();
	posA = Vector(0,0,1);
	posS = Vector(distance,0,1);
	mobilityAp->SetPosition(posA);
	mobilitySta->SetPosition(posS);
	
	// creating the building
	Ptr<Building> building = CreateObject<Building> ();
	building->SetBoundaries (Box (1.0, 50, -50.0, 50, 0, 300));
	building->SetBuildingType (Building::Residential);
	building->SetExtWallsType (Building::ConcreteWithWindows);
	Ptr<MobilityBuildingInfo> buildingInfo = CreateObject<MobilityBuildingInfo> ();
	BuildingsHelper::Install(apNode);  // building info added to nodes
	BuildingsHelper::Install(staNode);  // building info added to nodes
	BuildingsHelper::MakeMobilityModelConsistent();
	YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

	channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	channel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel");
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
	phy.SetChannel (channel.Create ());
	phy.Set("TxPowerStart", DoubleValue(txPwr));
	phy.Set("TxPowerEnd", DoubleValue(txPwr));
	WifiHelper wifi = WifiHelper::Default();
	wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
	NqosWifiMacHelper mac = NqosWifiMacHelper::Default();
	if (frequency == 2400)
	{
		wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	} else
	{
		wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
	}
	std::ostringstream oss;
	oss << "HtMcs" << 7; // 1 stream, 64QAM, coding rate 5/6
	Ssid ssid = Ssid ("myWifi");
	NetDeviceContainer apDevice;
	NetDeviceContainer staDevice;
	mac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid));
	apDevice = wifi.Install (phy, mac, apNode);
	mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue (false));
	staDevice = wifi.Install (phy, mac, staNode);
	// Internet stack
	InternetStackHelper stack;
	stack.Install (apNode);
	stack.Install (staNode);
	std::cout << "Distance = " << distance << " m" << std::endl;
	Ipv4AddressHelper address;

	address.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer staNodeInterface;
	Ipv4InterfaceContainer apNodeInterface;
	apNodeInterface = address.Assign(apDevice);
	staNodeInterface = address.Assign(staDevice);
	std::cout << "Path loss: " << hybridBuildingsPropagationLossModel->GetLoss(mobilityAp, mobilitySta) << "dBm" << std::endl;
	ApplicationContainer serverApp, sinkApp;
	UdpServerHelper myServer (9);
	serverApp = myServer.Install (staNode);
	serverApp.Start(Seconds(0));
	serverApp.Stop(Seconds(12));
	UdpClientHelper myClient (staNodeInterface.GetAddress (0), 9);
	myClient.SetAttribute("MaxPackets", UintegerValue(4294967295u));
	myClient.SetAttribute("Interval", TimeValue(Time("0.00001")));
	myClient.SetAttribute("PacketSize", UintegerValue(1472));
	ApplicationContainer clientApp = myClient.Install (apNode.Get (0));
	clientApp.Start(Seconds(1.0));
	clientApp.Stop(Seconds(11));
	Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
	Simulator::Stop(Seconds(12));
	Simulator::Run();
	Simulator::Destroy();

	double throughput = 0;
	uint32_t totalPacketsThrough;
	totalPacketsThrough = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
	throughput = totalPacketsThrough * 1472 * 8 / (10*1000.0); // kbit/s
	if (!printOnScreen)
	{
		outFile << throughput << " ";
	}
		std::cout << "Distance = " << distance << " m and throughput = " << throughput << "kbit/s" << std::endl;
	return 0;
} 
