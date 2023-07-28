/**************************************************************************************

***************************************************************************************/

#ifndef HDRHANDLER_H
#define HDRHANDLER_H

//#include "Communication_IF.h"
#include "communication_2206.h"
#include "communication.h"
#include "roboscan_nsl2206_image.h"
#include "roboscan_nsl2206_distance_image.h"
#include "roboscan_nsl2206_distance_amplitude_image.h"
#include "roboscan_nsl2206_distance_grayscale_image.h"
#include "roboscan_nsl2206_grayscale_image.h"



class HdrHandler2206
{
	public:

      HdrHandler2206();
	  ~HdrHandler2206();
	
      void processTemporalHdr(std::shared_ptr<ComLib::Nsl2206Image> &image);

  //public slots:
      void onDistanceReceived(std::shared_ptr<ComLib::Nsl2206Image>& image);
      void onDistanceAmplitudeReceived(std::shared_ptr<ComLib::Nsl2206Image>& image);
      void onDistanceGrayscaleReceived(std::shared_ptr<ComLib::Nsl2206Image>& image);

  //signals:
      void receivedDistance(std::shared_ptr<ComLib::Nsl2206Image>& image);
      void receivedDistanceAmplitude(std::shared_ptr<ComLib::Nsl2206Image>& image);
      void receivedDistanceGrayscale(std::shared_ptr<ComLib::Nsl2206Image>& image);

  	private:
      void patchImage(std::shared_ptr<ComLib::Nsl2206Image> &imageTarget, std::shared_ptr<ComLib::Nsl2206Image>& imageCompare);
      bool allImagesFilled(const unsigned int numImage);

      static const unsigned int NUM_HDR_IMAGE = 4;

      std::shared_ptr<ComLib::Nsl2206Image> hdrImage[NUM_HDR_IMAGE];
      std::shared_ptr<ComLib::Nsl2206Image> resultImage;
      unsigned int indexIntegrationTime;
	
};

#endif // HDRHANDLER_H
