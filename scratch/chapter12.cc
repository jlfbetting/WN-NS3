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
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-model.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"
#include "ns3/buildings-helper.h"
#include "ns3/hybrid-buildings-propagation-loss-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/kun-2600-mhz-propagation-loss-model.h"
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace ns3;
using std::vector;
NS_LOG_COMPONENT_DEFINE ("NS3wifiAssignment");
int main (int argc, char *argv[])
{
	double hAp = 1;  // vertical position of AP
	double hNode = 1; // vertical position of the node;
	double stepSize = 0.5;   // delta d for measuring
	double maxDistance = 100; // max distance for measuring
	int lossModel = 5;
	int powerAp = 0;     // power [dBm] of the AP signal
	int frequency = 5000;  // frequency [MHz]
	int systemLoss = 1;
	bool printOnScreen = false;
	int environmentOption = 3;
	std::string outputFilename = "myFile.out";
	CommandLine cmd;

	cmd.AddValue("lossModel", "Loss model number (1 = Log-distance model, 2 = Two-ray model, 3 = Friis model for vacuum, 4 = Kun 2600MHz model, 5 = Hybrid Buildings Propagation Loss model", lossModel);
	cmd.AddValue("frequency", "Frequency of the signal [MHz]", frequency);
	cmd.AddValue("printOnScreen", "Print on screen (true or false, 1 or 0)", printOnScreen);
	cmd.AddValue("outputFilename", "Filename of output file (incl. extension)", outputFilename);  
	cmd.AddValue("maxDistance", "The maximal distance from the AP that is measured", maxDistance);
	cmd.AddValue("stepSize", "The step size [m] that is taken into account", stepSize);
	cmd.AddValue("environmentOption", "The environment chosen (see report)", environmentOption);
	ConfigStore inputConfig;
	inputConfig.ConfigureDefaults ();
	cmd.Parse(argc, argv);

	// create and open the output file


	std::ofstream outFile;
	outFile.open(outputFilename.c_str());
	if (!outFile.is_open ())
	{
		NS_FATAL_ERROR ("Cannot open the output file");
	}
	// Path loss models
	Ptr<LogDistancePropagationLossModel> logDistancePropagationLossModel = CreateObject<LogDistancePropagationLossModel> ();
	Ptr<TwoRayGroundPropagationLossModel> twoRayGroundPropagationLossModel = CreateObject<TwoRayGroundPropagationLossModel> ();
	Ptr<FriisPropagationLossModel> friisPropagationLossModel = CreateObject<FriisPropagationLossModel> ();
	Ptr<Kun2600MhzPropagationLossModel> kun2600MhzPropagationLossModel = CreateObject<Kun2600MhzPropagationLossModel> ();
	Ptr<HybridBuildingsPropagationLossModel> hybridBuildingsPropagationLossModel = CreateObject<HybridBuildingsPropagationLossModel> ();
	if (lossModel == 2)
	{ 
		std::cout << "Loss model: TwoRayGroundPropagationLossModel" << std::endl;
		twoRayGroundPropagationLossModel->SetFrequency(frequency * pow(10,6));  // WiFi
		std::cout << "Frequency: " << twoRayGroundPropagationLossModel->GetFrequency() << std::endl;
		twoRayGroundPropagationLossModel->SetHeightAboveZ(0);  // z-coordinate is antenna
	} else if (lossModel == 3) {
		std::cout << "Loss model: Friis model" << std::endl;
		friisPropagationLossModel->SetFrequency(frequency * pow(10,6));
		friisPropagationLossModel->SetSystemLoss(systemLoss);
		std::cout << "Frequency: " << friisPropagationLossModel->GetFrequency() << std::endl;
		std::cout << "System loss: " << friisPropagationLossModel->GetSystemLoss() << std::endl;
		std::cout << "Min loss: " << friisPropagationLossModel->GetMinLoss() << std::endl;

	} else if (lossModel == 4) {
		std::cout << "Loss model: Kun 2600 Mhz" << std::endl;
	} else if (lossModel == 1) { 
		std::cout << "LogDistancePropagationLossModel" << std::endl;
		logDistancePropagationLossModel->SetReference(1,0);
		logDistancePropagationLossModel->SetPathLossExponent(2);
	}  else	{
		std::cout << "Hybrid Building Propagation Loss Model" << std::endl;	
		hybridBuildingsPropagationLossModel->SetFrequency(frequency * pow(10,6));
	}

	// shadowing / building loss models
	
	if (lossModel >= 5 )
	{
		if (environmentOption == 3)
		{
			std::cout << "Mobility option 3 chosen" << std::endl;
			Ptr<Building> building = CreateObject<Building> ();
			building->SetBoundaries (Box (-50, 50, -50, 50, 0, 50));
			building->SetBuildingType (Building::Residential);
			building->SetExtWallsType (Building::ConcreteWithWindows);
		} else if (environmentOption == 1)
		{
			std::cout << "Mobility option 1 chosen" << std::endl;
			Ptr<Building> building = CreateObject<Building> ();
			building->SetBoundaries (Box (1.0, 50, -50.0, 50, 0, 300));
			building->SetBuildingType (Building::Residential);
			building->SetExtWallsType (Building::ConcreteWithWindows);
		} else {
			std::cout << "Mobility option 2 chosen" << std::endl;
			Ptr<Building> building = CreateObject<Building> ();
			building->SetBoundaries (Box (-4.5, 4.5, -4.5, 4.5, 0, 4.5));
			building->SetBuildingType (Building::Residential);
			building->SetExtWallsType (Building::ConcreteWithWindows);
		}
	} 
	
	// install AP on grid
	Ptr<ConstantPositionMobilityModel> posAp = CreateObject<ConstantPositionMobilityModel> ();
	posAp->SetPosition (Vector (0.0, 0.0, hAp));
	Ptr<MobilityBuildingInfo> buildingInfoAp = CreateObject<MobilityBuildingInfo> ();
	posAp->AggregateObject (buildingInfoAp);
	BuildingsHelper::MakeConsistent(posAp);
	for (double i = maxDistance; i >= -maxDistance; i -= stepSize) // i -> direction y
	{  
		for (double j = -maxDistance; j <= maxDistance; j += stepSize) // j -> direction x
		{ 
			Ptr<ConstantPositionMobilityModel> posNode = CreateObject<ConstantPositionMobilityModel> ();
			posNode->SetPosition (Vector (j, i, hNode));
			Ptr<MobilityBuildingInfo> buildingInfoNode = CreateObject<MobilityBuildingInfo> ();
			posNode->AggregateObject (buildingInfoNode);
			BuildingsHelper::MakeConsistent(posNode);
			double loss = 0;
			if (lossModel == 2) {
				loss = -twoRayGroundPropagationLossModel->CalcRxPower(powerAp, posAp, posNode);  
			} else if (lossModel == 3) { 
				loss = -friisPropagationLossModel->CalcRxPower(powerAp, posAp, posNode);
			} else if (lossModel == 4) {
				loss = -kun2600MhzPropagationLossModel->CalcRxPower(powerAp, posAp, posNode);
			} else if (lossModel == 1) {
				loss = -logDistancePropagationLossModel->CalcRxPower(powerAp, posAp, posNode);
			} else {
				loss = hybridBuildingsPropagationLossModel->GetLoss(posAp, posNode);
			}
			if (!printOnScreen)
			{
				outFile << loss << " ";
			} else {
				std::cout << std::setfill(' ') << std::setw(3) << (int)round(loss) << " ";
			}       
		}
		if (!printOnScreen)
		{

			std::cout << "Row " << -(i-maxDistance)/stepSize+1 << " of " << maxDistance*2/stepSize+1 << " has been written." << std::endl;
			outFile << std::endl;
		} else {
			std::cout << std::endl;
		}
	} 
	Simulator::Destroy ();

	return 0;
} 
