#include <ros/ros.h>
#include <numeric>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "player_eeg/EEGSpectralData.h"

const uint32_t NumElectrodes = 14;

namespace BrainWaves
{
   enum BrainWave
   {
      Delta = 0,
      Theta,
      Alpha,
      Beta,
      Gamma,
      Count
   };
}

const float BrainWaveFreqUpperBound[BrainWaves::Count] =
{
   4.5,  //Delta ~0-4Hz
   7.5,  //Theta ~4-7Hz
   12.5, //Alpha ~8-12Hz
   30.5, //Beta  ~13-30Hz
   100   //Gamma ~31-100Hz
};

const float BrainWavePowerUpperBound[BrainWaves::Count] =
{
   50.0, // Delta
   10.0, // Theta
   10.0, // Alpha
   5.0,  // Beta
   1.0   // Gamma
};

float brainWavePower[BrainWaves::Count][NumElectrodes];

void AddHeadMeshMarker(visualization_msgs::MarkerArray& markerArray)
{
   const char* headModel = "package://data_visualizer/assets/random_head.dae";

   markerArray.markers.push_back(visualization_msgs::Marker());
   visualization_msgs::Marker& headMarker = markerArray.markers.back();

   // Create and add the head mesh marker
   headMarker.header.frame_id = "/map";
   headMarker.header.stamp = ros::Time::now();

   headMarker.ns = "";
   headMarker.id = 0;
   headMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
   headMarker.mesh_resource = headModel;
   headMarker.action = visualization_msgs::Marker::ADD;

   headMarker.pose.position.x = 0;
   headMarker.pose.position.y = 0;
   headMarker.pose.position.z = 0;
   headMarker.pose.orientation.x = 0.0f;
   headMarker.pose.orientation.y = 0.0f;
   headMarker.pose.orientation.z = 0.0f;
   headMarker.pose.orientation.w = 1.0f;

   headMarker.scale.x = 1.0f;
   headMarker.scale.y = 1.0f;
   headMarker.scale.z = 1.0f;

   headMarker.color.r = 0.5f;
   headMarker.color.g = 0.5f;
   headMarker.color.b = 0.5f;
   headMarker.color.a = 0.5f;

   headMarker.lifetime = ros::Duration();
}

std::string GetElectrodeName(int index)
{
   switch(index)
   {
   case 0:
      return "AF3";
      break;
   case 1:
      return "F7";
      break;
   case 2:
      return "F3";
      break;
   case 3:
      return "FC5";
      break;
   case 4:
      return "T7";
      break;
   case 5:
      return "P7";
      break;
   case 6:
      return "O1";
      break;
   case 7:
      return "O2";
      break;
   case 8:
      return "P8";
      break;
   case 9:
      return "T8";
      break;
   case 10:
      return "FC6";
      break;
   case 11:
      return "F4";
      break;
   case 12:
      return "F8";
      break;
   case 13:
      return "AF4";
      break;
   default:
      ROS_ERROR("%s", "Unknown index in GetElectrodeName(). Fix me!");
      break;
   }
}

float GetElectrodePower(int index, BrainWaves::BrainWave brainWave)
{
    return brainWavePower[brainWave][index];
}

void GetElectrodePosOnModel(int index, float& posX, float& posY, float& posZ)
{
   static float electrodePosOnModel[NumElectrodes][3] =
   {
      {24.6043, -26.5649, 140.4051},  // AF3
      {64.5430, 0.5480, 98.6852},     // F7
      {28.0879, 22.0239, 175.0008},   // F3
      {68.5718, 22.8922, 120.6030},   // FC5
      {73.6532, 59.0386, 92.5422},    // T7
      {68.5718, 76.1284, 120.4887},   // P7
      {37.1192, 140.0839, 120.4487},  // O1
      {-37.1192, 140.0839, 120.4487}, // O2
      {-68.5718, 76.1284, 120.4887},  // P8
      {-73.6532, 59.0386, 92.5422},   // T8
      {-68.5718, 22.8922, 120.6030},  // FC6
      {-28.0879, 22.0239, 175.0008},  // F4
      {-64.5430, 0.5480, 98.6852},    // F8
      {-24.6043, -26.5649, 140.4051}, // AF4
   };

   posX = electrodePosOnModel[index][0];
   posY = electrodePosOnModel[index][1];
   posZ = electrodePosOnModel[index][2];
}

void GetElectrodeColor(float brainWavePower,
                       BrainWaves::BrainWave brainWave,
                       float& red, float& green, float& blue)
{
   float normalizedPower = brainWavePower/BrainWavePowerUpperBound[brainWave];

   // Green-to-red color gradient
   normalizedPower = std::max(0.0f, std::min(normalizedPower, 1.0f));
   red = normalizedPower;
   green = 1.0 - normalizedPower;
   blue = 0.0;
}

void AddElectrodeMarkers(visualization_msgs::MarkerArray& markerArray)
{
   int32_t startingID = std::max((int32_t)markerArray.markers.size(), 1);
   for (int i = 0; i < NumElectrodes; ++i)
   {
      float posX, posY, posZ;
      float red, green, blue;
      float brainWavePower = GetElectrodePower(i, BrainWaves::Alpha);
      std::string electrodeName = GetElectrodeName(i);
      GetElectrodePosOnModel(i, posX, posY, posZ);
      GetElectrodeColor(brainWavePower, BrainWaves::Alpha, red, green, blue);

      // Create and add electrode sphere marker
      markerArray.markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& marker = markerArray.markers.back();
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "";
      marker.id = startingID + (2*i);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = posX;
      marker.pose.position.y = posY;
      marker.pose.position.z = posZ;
      marker.pose.orientation.x = 0.0f;
      marker.pose.orientation.y = 0.0f;
      marker.pose.orientation.z = 0.0f;
      marker.pose.orientation.w = 1.0f;
      marker.scale.x = 15.0f;
      marker.scale.y = 15.0f;
      marker.scale.z = 15.0f;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      marker.color.a = 1.0f;
      marker.lifetime = ros::Duration();

/*
      // Create and add electrode text marker to display the brain wave power value
      markerArray.markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& textMarker = markerArray.markers.back();
      textMarker.header.frame_id = "/map";
      textMarker.header.stamp = ros::Time::now();
      textMarker.ns = "";
      textMarker.id = startingID + (2*i + 1);
      textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      textMarker.action = visualization_msgs::Marker::ADD;
      textMarker.pose.position.x = 1.2*posX;
      textMarker.pose.position.y = 1.2*posY;
      textMarker.pose.position.z = 1.2*posZ;
      textMarker.scale.x = 15.0f;
      textMarker.scale.y = 15.0f;
      textMarker.scale.z = 15.0f;
      textMarker.color.r = 1.0f;
      textMarker.color.g = 0.0f;
      textMarker.color.b = 0.0f;
      textMarker.color.a = 1.0f;
      textMarker.lifetime = ros::Duration();
      std::stringstream stringStream;
      //stringStream << brainWavePower;
      stringStream << electrodeName;
      textMarker.text = stringStream.str();
*/
   }
}

void EEGSpectralCallback(const player_eeg::EEGSpectralData::ConstPtr& eegSpectralData)
{
   BrainWaves::BrainWave brainWave = BrainWaves::Delta;
   std::vector<float>::const_iterator lowerFreqIter = eegSpectralData->freqs.begin();
   while (lowerFreqIter != eegSpectralData->freqs.end() && brainWave != BrainWaves::Count)
   {
      std::vector<float>::const_iterator upperFreqIter = std::upper_bound(lowerFreqIter, eegSpectralData->freqs.end(), BrainWaveFreqUpperBound[(int)brainWave]);
      int32_t lowerIndex = lowerFreqIter - eegSpectralData->freqs.begin();
      int32_t upperIndex = upperFreqIter - eegSpectralData->freqs.begin();
      float powerSum = std::accumulate(eegSpectralData->af3.begin()+lowerIndex, eegSpectralData->af3.begin()+upperIndex, 0);
      brainWavePower[brainWave][0] = powerSum;
      powerSum = std::accumulate(eegSpectralData->f7.begin()+lowerIndex, eegSpectralData->f7.begin()+upperIndex, 0);
      brainWavePower[brainWave][1] = powerSum;
      powerSum = std::accumulate(eegSpectralData->f3.begin()+lowerIndex, eegSpectralData->f3.begin()+upperIndex, 0);
      brainWavePower[brainWave][2] = powerSum;
      powerSum = std::accumulate(eegSpectralData->fc5.begin()+lowerIndex, eegSpectralData->fc5.begin()+upperIndex, 0);
      brainWavePower[brainWave][3] = powerSum;
      powerSum = std::accumulate(eegSpectralData->t7.begin()+lowerIndex, eegSpectralData->t7.begin()+upperIndex, 0);
      brainWavePower[brainWave][4] = powerSum;
      powerSum = std::accumulate(eegSpectralData->p7.begin()+lowerIndex, eegSpectralData->p7.begin()+upperIndex, 0);
      brainWavePower[brainWave][5] = powerSum;
      powerSum = std::accumulate(eegSpectralData->o1.begin()+lowerIndex, eegSpectralData->o1.begin()+upperIndex, 0);
      brainWavePower[brainWave][6] = powerSum;
      powerSum = std::accumulate(eegSpectralData->o2.begin()+lowerIndex, eegSpectralData->o2.begin()+upperIndex, 0);
      brainWavePower[brainWave][7] = powerSum;
      powerSum = std::accumulate(eegSpectralData->p8.begin()+lowerIndex, eegSpectralData->p8.begin()+upperIndex, 0);
      brainWavePower[brainWave][8] = powerSum;
      powerSum = std::accumulate(eegSpectralData->t8.begin()+lowerIndex, eegSpectralData->t8.begin()+upperIndex, 0);
      brainWavePower[brainWave][9] = powerSum;
      powerSum = std::accumulate(eegSpectralData->fc6.begin()+lowerIndex, eegSpectralData->fc6.begin()+upperIndex, 0);
      brainWavePower[brainWave][10] = powerSum;
      powerSum = std::accumulate(eegSpectralData->f4.begin()+lowerIndex, eegSpectralData->f4.begin()+upperIndex, 0);
      brainWavePower[brainWave][11] = powerSum;
      powerSum = std::accumulate(eegSpectralData->f8.begin()+lowerIndex, eegSpectralData->f8.begin()+upperIndex, 0);
      brainWavePower[brainWave][12] = powerSum;
      powerSum = std::accumulate(eegSpectralData->af4.begin()+lowerIndex, eegSpectralData->af4.begin()+upperIndex, 0);
      brainWavePower[brainWave][13] = powerSum;
      lowerFreqIter = upperFreqIter;
      brainWave = (BrainWaves::BrainWave)((int)brainWave + 1);
   }

   //ROS_ERROR("%f %f %f %f %f", brainWavePower[0][0], brainWavePower[1][0], brainWavePower[2][0], brainWavePower[3][0], brainWavePower[4][0]);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "visualizerPlayerEEG");
   ros::NodeHandle nodeHandle;
   ros::Subscriber eegSpectralSub = nodeHandle.subscribe("/player_eeg_spectral", 1, EEGSpectralCallback);
   ros::Publisher eegMarkerPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("/visualization/player_eeg", 1);

   ros::Duration updateHeadMeshDuration(1.0);
   ros::Time updateHeadMeshTimer = ros::Time::now();
   while(ros::ok())
   {
      visualization_msgs::MarkerArray markerArray;
      if (ros::Time::now() >= updateHeadMeshTimer + updateHeadMeshDuration)
      {
         AddHeadMeshMarker(markerArray);
         updateHeadMeshTimer = ros::Time::now();
      }
      AddElectrodeMarkers(markerArray);
      eegMarkerPub.publish(markerArray);

      ros::spinOnce();
   }
}
