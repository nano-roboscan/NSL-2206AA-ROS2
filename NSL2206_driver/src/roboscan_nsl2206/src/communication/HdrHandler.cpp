/**************************************************************************************

***************************************************************************************/

#include "HdrHandler.h"

using namespace ComLib;

HdrHandler2206::HdrHandler2206()
{
  indexIntegrationTime = 0;
}
HdrHandler2206::~HdrHandler2206()
{
}

/**
 * @brief Patch image
 *
 * Runs through all pixels of the first image and changes the data, if the second image contains better data.
 *
 * @param imageTarget Reference to the target image to possible change the pixel
 * @param imageCompare Image with possible better data (higher integration time)
  */
void HdrHandler2206::patchImage(std::shared_ptr<Nsl2206Image> &imageTarget, std::shared_ptr<Nsl2206Image>& imageCompare)
{
  unsigned int numPixel = imageTarget->getNumPixel();

  for (unsigned int i = 0; i < numPixel; i++)
  {
    if (imageCompare->getPixelIsInvalid(imageCompare->getDistanceOfPixel(i)) == false)
    {
      imageTarget.get()->changePixel(*(imageCompare.get()), i);
    }
  }
}

/**
 * @brief Check, if all images in the buffer contain data
 *
 * @param numImage Number of images needed for HDR = number of integration times used
 * @retval false Not all immages contain data
 * @retval true All images contain data
 * @return Status, if all images for a HDR image contain data
 */
bool HdrHandler2206::allImagesFilled(const unsigned int numImage)
{
  for (unsigned int i = 0; i < numImage; i++)
  {
    if (hdrImage[i].get() == nullptr)
    {
      return false;
    }
  }

  return true;
}

/**
 * @brief Process temporal HDR
 *
 * This function patches multiple images to one image to generate a HDR image.
 *
 * @param image Reference to the received image. This image will be changed to a HDR image if needed.
 */
void HdrHandler2206::processTemporalHdr(std::shared_ptr<Nsl2206Image> &image)
{
  unsigned int numIntegrationTimeUsed = image->getNumIntegrationTimeUsed();

  //Add the image to the buffer
  if (indexIntegrationTime < NUM_HDR_IMAGE)
  {
    hdrImage[indexIntegrationTime] = image;
  }

  indexIntegrationTime++;

  //Last image received. Now patch them to one image.
  if ((allImagesFilled(numIntegrationTimeUsed)) &&  (indexIntegrationTime >= numIntegrationTimeUsed))
  {
    indexIntegrationTime = 0;
    resultImage = hdrImage[0];

    for (unsigned int i = 1; i < numIntegrationTimeUsed; i++)
    {
      patchImage(resultImage, hdrImage[i]);
    }
  }

  //Change to patched image only if there are enough images received
  if ((allImagesFilled(numIntegrationTimeUsed)) && (resultImage.get() != nullptr))
  {
    image = resultImage;
  }
}

void HdrHandler2206::onDistanceReceived(std::shared_ptr<Nsl2206Image>& image)
{
  if (image->getHeaderFlags().getTemporalHdrEnabled())
  {
    processTemporalHdr(image);
  }
  
  //emit receivedDistance(image);
}

void HdrHandler2206::onDistanceAmplitudeReceived(std::shared_ptr<Nsl2206Image>& image)
{
  if (image->getHeaderFlags().getTemporalHdrEnabled())
  {
    processTemporalHdr(image);
  }

  //emit receivedDistanceAmplitude(image);
}

void HdrHandler2206::onDistanceGrayscaleReceived(std::shared_ptr<Nsl2206Image>& image)
{
  if (image->getHeaderFlags().getTemporalHdrEnabled())
  {
    processTemporalHdr(image);
  }

  //emit receivedDistanceGrayscale(image);
}

