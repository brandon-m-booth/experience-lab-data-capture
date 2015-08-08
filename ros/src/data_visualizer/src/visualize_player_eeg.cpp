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

void AddElectrodeMarkers(visualization_msgs::MarkerArray& markerArray)
{
   float electrodePosOnModel[NumElectrodes][3] =
   {
      {-24.6043, -26.5649, 140.4051},
      {-28.0879, 22.0239, 175.0008},
      {-64.5430, 0.5480, 98.6852},
      {-68.5718, 22.8922, 120.6030},
      {-73.6532, 59.0386, 92.5422},
      {-68.5718, 76.1284, 120.4887},
      {-37.1192, 140.0839, 120.4487},
      // Below, these are mirrored about the X-axis
      {24.6043, -26.5649, 140.4051},
      {28.0879, 22.0239, 175.0008},
      {64.5430, 0.5480, 98.6852},
      {68.5718, 22.8922, 120.6030},
      {73.6532, 59.0386, 92.5422},
      {68.5718, 76.1284, 120.4887},
      {37.1192, 140.0839, 120.4487},
   };

   int32_t startingID = markerArray.markers.size();
   for (int i = 0; i < NumElectrodes; ++i)
   {
      markerArray.markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& marker = markerArray.markers.back();
      markerArray.markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& textMarker = markerArray.markers.back();

      // Create and add electrode sphere marker
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();

      marker.ns = "";
      marker.id = startingID + (2*i);
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = electrodePosOnModel[i][0];
      marker.pose.position.y = electrodePosOnModel[i][1];
      marker.pose.position.z = electrodePosOnModel[i][2];
      marker.pose.orientation.x = 0.0f;
      marker.pose.orientation.y = 0.0f;
      marker.pose.orientation.z = 0.0f;
      marker.pose.orientation.w = 1.0f;

      marker.scale.x = 15.0f;
      marker.scale.y = 15.0f;
      marker.scale.z = 15.0f;

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.lifetime = ros::Duration();

      textMarker = marker;
      textMarker.header.frame_id = "/map";
      textMarker.header.stamp = ros::Time::now();
      textMarker.id = startingID + (2*i + 1);
      textMarker.pose.position.x *= 1.2;
      textMarker.pose.position.y *= 1.2;
      textMarker.pose.position.z *= 1.2;
      textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      std::stringstream stringStream;
      stringStream << brainWavePower[BrainWaves::Alpha][i];
      textMarker.text = stringStream.str();
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
      float powerSum = std::accumulate(eegSpectralData->e1.begin()+lowerIndex, eegSpectralData->e1.begin()+upperIndex, 0);
      brainWavePower[brainWave][0] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e2.begin()+lowerIndex, eegSpectralData->e2.begin()+upperIndex, 0);
      brainWavePower[brainWave][1] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e3.begin()+lowerIndex, eegSpectralData->e3.begin()+upperIndex, 0);
      brainWavePower[brainWave][2] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e4.begin()+lowerIndex, eegSpectralData->e4.begin()+upperIndex, 0);
      brainWavePower[brainWave][3] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e5.begin()+lowerIndex, eegSpectralData->e5.begin()+upperIndex, 0);
      brainWavePower[brainWave][4] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e6.begin()+lowerIndex, eegSpectralData->e6.begin()+upperIndex, 0);
      brainWavePower[brainWave][5] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e7.begin()+lowerIndex, eegSpectralData->e7.begin()+upperIndex, 0);
      brainWavePower[brainWave][6] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e8.begin()+lowerIndex, eegSpectralData->e8.begin()+upperIndex, 0);
      brainWavePower[brainWave][7] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e9.begin()+lowerIndex, eegSpectralData->e9.begin()+upperIndex, 0);
      brainWavePower[brainWave][8] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e10.begin()+lowerIndex, eegSpectralData->e10.begin()+upperIndex, 0);
      brainWavePower[brainWave][9] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e11.begin()+lowerIndex, eegSpectralData->e11.begin()+upperIndex, 0);
      brainWavePower[brainWave][10] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e12.begin()+lowerIndex, eegSpectralData->e12.begin()+upperIndex, 0);
      brainWavePower[brainWave][11] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e13.begin()+lowerIndex, eegSpectralData->e13.begin()+upperIndex, 0);
      brainWavePower[brainWave][12] = powerSum;
      powerSum = std::accumulate(eegSpectralData->e14.begin()+lowerIndex, eegSpectralData->e14.begin()+upperIndex, 0);
      brainWavePower[brainWave][13] = powerSum;
      lowerFreqIter = upperFreqIter;
      brainWave = (BrainWaves::BrainWave)((int)brainWave + 1);
   }

   ROS_ERROR("%f %f %f %f %f", brainWavePower[0][0], brainWavePower[1][0], brainWavePower[2][0], brainWavePower[3][0], brainWavePower[4][0]);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "visualizerPlayerEEG");
   ros::NodeHandle nodeHandle;
   ros::Subscriber eegSpectralSub = nodeHandle.subscribe("/player_eeg_spectral", 1, EEGSpectralCallback);
   ros::Publisher eegMarkerPub = nodeHandle.advertise<visualization_msgs::MarkerArray>("/visualization/player_eeg", 1);

   int32_t prevNumSubscribers = 0;
   while(ros::ok())
   {
      int32_t curNumSubscribers = eegMarkerPub.getNumSubscribers();
      if (curNumSubscribers != 0)
      {
         visualization_msgs::MarkerArray markerArray;
         if (curNumSubscribers != prevNumSubscribers)
         {
            AddHeadMeshMarker(markerArray);
            curNumSubscribers = prevNumSubscribers;
         }
         AddElectrodeMarkers(markerArray);
         eegMarkerPub.publish(markerArray);
      }
      else
      {
         prevNumSubscribers = 0;
      }

      ros::spinOnce();
   }
}
