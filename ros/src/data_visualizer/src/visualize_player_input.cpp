#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "player_input/CursorPosition.h"
#include <string.h>
#include <sstream>
#include <algorithm>
#include <stdio.h>
#include <png.h>

// Global structures
struct Pixel
{
   int x;
   int y;
};

struct KeyMapping
{
   inline bool operator==(const KeyMapping& other) const {return this->keyName == other.keyName;}
   inline bool operator<(const KeyMapping& other) const {return this->keyName < other.keyName;}

   std::string keyName;
   Pixel upperLeftPixel;
   Pixel lowerRightPixel;
};

// Global variables
sensor_msgs::Image keysNormalImage;
sensor_msgs::Image keysPressedImage;
sensor_msgs::Image keyStateImage;
ros::Time cleanUpKeyStateTime;
bool needCleanUpKeyState = false;
bool hasKeyStateChanged = true;

// Forward declarations
bool LoadPNG(sensor_msgs::Image& image, std::string fileName);
bool GetBoundingBoxFromKeyEvent(Pixel& upperLeftPixel, Pixel& lowerRightPixel, const std::string& keyEventName, const std::string& keyState);
void BlitKeyboardPixels(const Pixel& upperLeftPixel, const Pixel& lowerRightPixel, const std::string& keyState);
void CleanUpKeyState();

void PlayerKeyEventCallback(const std_msgs::String::ConstPtr& playerKeyEvent)
{
   // Parse the event into the key name and key state substrings
   std::string playerEventString = playerKeyEvent->data;
   std::istringstream stringStream(playerEventString);
   std::string keyEventName, keyState;
   stringStream >> keyEventName;
   stringStream >> keyState;

   // Get the bounding box of the corresponding key in the keyboard image
   Pixel upperLeftPixel, lowerRightPixel;
   if (GetBoundingBoxFromKeyEvent(upperLeftPixel, lowerRightPixel, keyEventName, keyState))
   {
      if (keyEventName == "MouseWheelScroll")
      {
         keyState = "Down"; // This makes the blit call below "press" the button
         cleanUpKeyStateTime = ros::Time::now() + ros::Duration(0.5);
         needCleanUpKeyState = true;
      }

      // Blit the pixels inside the bounding box into keyStateImage
      BlitKeyboardPixels(upperLeftPixel, lowerRightPixel, keyState);

      hasKeyStateChanged = true;
   }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "visualizerPlayerInput");
	ros::NodeHandle nodeHandle;
	ros::Subscriber playerKeyEventsSub = nodeHandle.subscribe("/playerInput/keyEvents", 1, PlayerKeyEventCallback);
   ros::Publisher playerKeyEventsPub = nodeHandle.advertise<sensor_msgs::Image>("/visualization/player_input_key_events", 1);
   ros::Duration maxPublishDuration(0.0333333); // 30 Hertz

   // Load the default keyboard image (all keys not pressed)
   std::string assetsPath = ros::package::getPath("data_visualizer")+"/assets/";
   std::string normalKeyboardFileName = assetsPath+"Keyboard.png";
   if (!LoadPNG(keysNormalImage, normalKeyboardFileName))
   {
      std::string errorString = "Unable to load PNG file " + normalKeyboardFileName;
      ROS_ERROR("%s", errorString.c_str());
   }

   std::string pressedKeyboardFileName = assetsPath+"KeyboardPressed.png";
   if (!LoadPNG(keysPressedImage, pressedKeyboardFileName))
   {
      std::string errorString = "Unable to load PNG file " + pressedKeyboardFileName;
      ROS_ERROR("%s", errorString.c_str());
   }

   keyStateImage = keysNormalImage;

   cleanUpKeyStateTime = ros::Time::now();

   while (ros::ok())
   {
      if (needCleanUpKeyState && (ros::Time::now() - cleanUpKeyStateTime > ros::Duration(0)))
      {
         CleanUpKeyState();
         needCleanUpKeyState = false;
      }

      if (hasKeyStateChanged)
      {
         playerKeyEventsPub.publish(keyStateImage);
         hasKeyStateChanged = false;
      }
      
      ros::Time startTime = ros::Time::now();
      while (ros::Time::now() - startTime < maxPublishDuration)
      {
         ros::spinOnce();
      }
   }
}

bool LoadPNG(sensor_msgs::Image& image, std::string fileName)
{
   png_structp pngPtr;
   png_infop infoPtr;
   FILE *fp;

   if ((fp = fopen(fileName.c_str(), "rb")) == NULL)
   {
       return false;
   }

   pngPtr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

   if (pngPtr == NULL)
   {
       fclose(fp);
       return false;
   }

   infoPtr = png_create_info_struct(pngPtr);
   if (infoPtr == NULL)
   {
       fclose(fp);
       png_destroy_read_struct(&pngPtr, NULL, NULL);
       return false;
   }

   if (setjmp(png_jmpbuf(pngPtr)))
   {
       png_destroy_read_struct(&pngPtr, &infoPtr, NULL);
       fclose(fp);
       return false;
   }

   png_init_io(pngPtr, fp);

   png_read_png(pngPtr, infoPtr, PNG_TRANSFORM_STRIP_16 | PNG_TRANSFORM_SWAP_ALPHA | PNG_TRANSFORM_EXPAND, NULL);

   png_uint_32 width = png_get_image_width(pngPtr, infoPtr);
   png_uint_32 height = png_get_image_height(pngPtr, infoPtr);

   png_uint_32 bitdepth   = png_get_bit_depth(pngPtr, infoPtr);
   png_uint_32 channels   = png_get_channels(pngPtr, infoPtr);
   png_uint_32 color_type = png_get_color_type(pngPtr, infoPtr);

   image.width = width;
   image.height = height;
   image.step = image.width * channels * bitdepth/8;
   if (channels == 3)
   {
      image.encoding = "rgb8";
   }
   else
   {
      ROS_ERROR("Unhandled encoding. FIX ME!");
   }
   image.data.resize(image.step*height);

   png_bytepp rowPointers;
   rowPointers = png_get_rows(pngPtr, infoPtr);

   for (unsigned int i = 0; i < height; ++i)
   {
      memcpy(&image.data[i*image.step], rowPointers[i], image.step);
   }

   png_destroy_read_struct(&pngPtr, &infoPtr, NULL);
   fclose(fp);

   return true;
}

bool GetBoundingBoxFromKeyEvent(Pixel& upperLeftPixel, Pixel& lowerRightPixel, const std::string& keyEventName, const std::string& keyState)
{
   static const KeyMapping keyMappings[] =
   {
      {"MouseLeft", {1463, 154}, {1493, 218}},
      {"MouseRight", {1537, 153}, {1567, 218}},
      {"MouseWheel", {1508, 169}, {1521, 200}},
      {"Backspace", {778, 116}, {892, 172}},
      {"Tab", {24, 175}, {108, 231}},
      {"Enter", {762, 234}, {892, 290}},
      {"Pause", {1051, 29}, {1114, 85}},
      {"CapsLock", {24, 234}, {121, 290}},
      {"Escape", {24, 29}, {79, 85}},
      {"Space", {249, 352}, {592, 407}},
      {"PageUp", {1051, 116}, {1114, 172}},
      {"PageDown", {1051, 175}, {1114, 231}},
      {"End", {985, 175}, {1048, 231}},
      {"Home", {985, 116}, {1048, 172}},
      {"Left", {931, 352}, {986, 407}},
      {"Up", {989, 293}, {1044, 349}},
      {"Right", {1047, 352}, {1102, 407}},
      {"Down", {989, 352}, {1044, 407}},
      {"PrintScreen", {919, 29}, {982, 85}},
      {"Insert", {919, 116}, {982, 172}},
      {"Delete", {919, 175}, {982, 231}},
      {"Zero", {604, 116}, {659, 172}},
      {"One", {82, 116}, {137, 172}},
      {"Two", {140, 116}, {195, 172}},
      {"Three", {198, 116}, {253, 172}},
      {"Four", {256, 116}, {311, 172}},
      {"Five", {314, 116}, {369, 172}},
      {"Six", {372, 116}, {427, 172}},
      {"Seven", {430, 116}, {485, 172}},
      {"Eight", {488, 116}, {543, 172}},
      {"Nine", {546, 116}, {601, 172}},
      {"A", {124, 234}, {179, 290}},
      {"B", {376, 293}, {431, 349}},
      {"C", {260, 293}, {315, 349}},
      {"D", {240, 234}, {295, 290}},
      {"E", {227, 175}, {282, 231}},
      {"F", {298, 234}, {353, 290}},
      {"G", {356, 234}, {411, 290}},
      {"H", {414, 234}, {469, 290}},
      {"I", {517, 175}, {572, 231}},
      {"J", {472, 234}, {527, 290}},
      {"K", {530, 234}, {585, 290}},
      {"L", {588, 234}, {643, 290}},
      {"M", {492, 293}, {545, 349}},
      {"N", {434, 293}, {489, 349}},
      {"O", {575, 175}, {630, 231}},
      {"P", {633, 175}, {688, 231}},
      {"Q", {111, 175}, {166, 231}},
      {"R", {285, 175}, {340, 231}},
      {"S", {182, 234}, {237, 290}},
      {"T", {343, 175}, {398, 231}},
      {"U", {459, 175}, {514, 231}},
      {"V", {318, 293}, {373, 349}},
      {"W", {169, 175}, {224, 231}},
      {"X", {202, 293}, {257, 349}},
      {"Y", {401, 175}, {456, 231}},
      {"Z", {144, 293}, {199, 349}},
      {"LeftSpecialKey", {111, 352}, {173, 407}},
      {"RightSpecialKey", {666, 352}, {728, 407}},
      {"ApplicationsKey", {731, 352}, {809, 407}},
      {"NumPad0", {1141, 352}, {1254, 408}},
      {"NumPad1", {1141, 293}, {1196, 349}},
      {"NumPad2", {1199, 293}, {1254, 349}},
      {"NumPad3", {1257, 293}, {1312, 349}},
      {"NumPad4", {1141, 234}, {1196, 290}},
      {"NumPad5", {1199, 234}, {1254, 290}},
      {"NumPad6", {1257, 234}, {1312, 290}},
      {"NumPad7", {1141, 175}, {1196, 231}},
      {"NumPad8", {1199, 175}, {1254, 231}},
      {"NumPad9", {1257, 175}, {1312, 231}},
      {"Multiply", {1257, 116}, {1312, 172}},
      {"Add", {1315, 175}, {1370, 290}},
      {"Subtract", {1315, 116}, {1370, 172}},
      {"Decimal", {1257, 352}, {1312, 408}},
      {"Divide", {1199, 116}, {1254, 172}},
      {"F1", {149, 29}, {204, 85}},
      {"F2", {207, 29}, {262, 85}},
      {"F3", {265, 29}, {320, 85}},
      {"F4", {323, 29}, {378, 85}},
      {"F5", {406, 29}, {461, 85}},
      {"F6", {464, 29}, {519, 85}},
      {"F7", {522, 29}, {577, 85}},
      {"F8", {580, 29}, {635, 85}},
      {"F9", {663, 29}, {718, 85}},
      {"F10", {721, 1}, {776, 85}},
      {"F11", {779, 1}, {834, 85}},
      {"F12", {837, 1}, {892, 85}},
      {"Numlock", {1141, 116}, {1196, 172}},
      {"ScrollLock", {985, 29}, {1048, 85}},
      {"LeftShift", {24, 293}, {141, 349}},
      {"RightShift", {724, 293}, {892, 349}},
      {"LeftControl", {24, 352}, {108, 407}},
      {"RightContol", {812, 352}, {892, 407}},
      {"LeftAlt", {176, 352}, {246, 407}},
      {"RightAlt", {595, 352}, {663, 407}},
      {"Semicolon", {646, 234}, {701, 290}},
      {"Equals", {720, 116}, {775, 172}},
      {"Comma", {550, 293}, {605, 349}},
      {"Minus", {662, 116}, {717, 172}},
      {"Period", {608, 293}, {663, 349}},
      {"ForwardSlash", {666, 293}, {721, 349}},
      {"Backtick", {24, 116}, {79, 172}},
      {"LeftBracket", {691, 175}, {746, 231}},
      {"BackSlash", {807, 175}, {892, 231}},
      {"RightBracket", {749, 175}, {804, 231}},
      {"Apostrophe", {704, 234}, {759, 290}}
   };
   KeyMapping mouseWheelScrollUp = {"MouseWheelScrollUp", {1506, 162}, {1523, 168}};
   KeyMapping mouseWheelScrollDown = {"MouseWheelScrollDown", {1506, 200}, {1522, 206}};

   static const size_t numKeyMappings = sizeof(keyMappings)/sizeof(KeyMapping);
   static std::vector<KeyMapping> keyMappingsSorted(keyMappings, keyMappings+numKeyMappings);

   // One-time setup
   static bool isSorted = false;
   if (!isSorted)
   {
      std::sort(keyMappingsSorted.begin(), keyMappingsSorted.end());
      isSorted = true;
   }

   KeyMapping targetKey;
   targetKey.keyName = keyEventName;
   std::vector<KeyMapping>::iterator iter = lower_bound(keyMappingsSorted.begin(), keyMappingsSorted.end(), targetKey);
   if (iter != keyMappingsSorted.end() && !(targetKey < *iter))
   {
      upperLeftPixel = iter->upperLeftPixel;
      lowerRightPixel = iter->lowerRightPixel;
      return true;
   }

   if (keyEventName == "MouseWheelScroll")
   {
      if (keyState[0] == '-')
      {
         upperLeftPixel = mouseWheelScrollDown.upperLeftPixel;
         lowerRightPixel = mouseWheelScrollDown.lowerRightPixel;
      }
      else
      {
         upperLeftPixel = mouseWheelScrollUp.upperLeftPixel;
         lowerRightPixel = mouseWheelScrollUp.lowerRightPixel;
      }
      return true;
   }

   return false;
}

void BlitKeyboardPixels(const Pixel& upperLeftPixel, const Pixel& lowerRightPixel, const std::string& keyState)
{
   sensor_msgs::Image* image = NULL;
   if (keyState == "Up")
   {
      image = &keysNormalImage;
   }
   else if (keyState == "Down")
   {
      image = &keysPressedImage;
   }
   else
   {
      ROS_ERROR("Unknown key state string. FIX ME!");
   }

   if (image)
   {
      int bytesPerPixel = keyStateImage.step / keyStateImage.width;
      for (int x = upperLeftPixel.x; x <= lowerRightPixel.x; ++x)
      {
         for (int y = upperLeftPixel.y; y <= lowerRightPixel.y; ++y)
         {
            size_t index = x*bytesPerPixel + y*keyStateImage.step;
            memcpy(&keyStateImage.data[index], &image->data[index], bytesPerPixel);
         }
      }
   }
}

void CleanUpKeyState()
{
   // Get the bounding box of the corresponding key in the keyboard image
   Pixel upperLeftPixel, lowerRightPixel;
   if (GetBoundingBoxFromKeyEvent(upperLeftPixel, lowerRightPixel, "MouseWheelScroll", "1"))
   {
      // Blit the pixels inside the bounding box into keyStateImage
      BlitKeyboardPixels(upperLeftPixel, lowerRightPixel, "Up");
   }

   if (GetBoundingBoxFromKeyEvent(upperLeftPixel, lowerRightPixel, "MouseWheelScroll", "-1"))
   {
      // Blit the pixels inside the bounding box into keyStateImage
      BlitKeyboardPixels(upperLeftPixel, lowerRightPixel, "Up");
   }

   hasKeyStateChanged = true;
}
