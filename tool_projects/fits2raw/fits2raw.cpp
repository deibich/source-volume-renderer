#include <fitsio.h>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>


bool checkFitsFile(fitsfile *dataFile) {

  int status = 0;

  // get number of "header-data-units"
  int numHDUs = 0;
  fits_get_num_hdus(dataFile,&numHDUs,&status);
  if (numHDUs == 0) {
    fprintf(stderr,"FITS error loading file, no HDUs present\n");
    fits_close_file(dataFile,&status);
    return false;
  }

  fprintf(stdout,"FITS number of HDUs: %i\n",numHDUs);

  int hduType = 0;
  fits_get_hdu_type(dataFile,&hduType,&status);
  if (hduType != IMAGE_HDU) {
    fprintf(stderr,"FITS unsupported format, 1st HDU must be IMAGE\n");
    fits_close_file(dataFile,&status);
    return false;
  }

  int naxis = 0;
  fits_get_img_dim(dataFile,&naxis,&status);
  if (naxis != 3) {
    fprintf(stderr,"FITS unsupported format, num dimensions must be 3\n");
    fits_close_file(dataFile,&status);
    return false;
  }

  return true;
}


template <class T>
void maskValues(T& values, fitsfile* maskFile, long naxes[3]) {
    std::vector<long> valuesMask(naxes[0] * size_t(naxes[1]) * naxes[2]);
    long fpixel[3] = { 1,1,1 };
    int status = 0;
    int retVal = fits_read_pix(maskFile, TLONG, fpixel, naxes[0] * size_t(naxes[1]) * naxes[2], 0,
        valuesMask.data(), 0, &status);
    fprintf(stdout, "FITS Mask retVal: %i, status: %i\n", retVal, status);
    bool firstLayerFound = false;
    size_t lastLayer = 0;
    for (size_t z = 0; z < naxes[2]; z++)
    {
      size_t z_offset = z * naxes[1] * naxes[0];
      for (size_t y = 0; y < naxes[1]; y++)
      {
        for (size_t x = 0; x < naxes[0]; x++)
        {
          size_t i = z_offset + y * naxes[0] + x;
          if (valuesMask[i] == 0) {
            values[i] = 0;
          }
          else
          {
            if (!firstLayerFound)
            {
              std::cout << "First Layer " << z << std::endl;
              firstLayerFound = true;
            }
            lastLayer = z;
          }
        }
      }
    }
    std::cout << "Last Layer " << lastLayer << std::endl;
    

    /*
    for (size_t i = 0; i < valuesMask.size(); ++i) {
        if (valuesMask[i] == 0) {
            values[i] = 0;
        }
    }
    */
}

int main(int ac, char **av)
{
  std::string inFileName = av[1];
  std::string varName = "unknown";
  std::string outFileBase = "./rawfits";
  std::string maskFileName = "";
  bool hasMask = false;
  bool normalize = false;

  bool outputAsBytes = false;

  bool chopZ = false;
  size_t startZ = 0;
  size_t endZ = 0;

  size_t width = 0;
  size_t height = 0;
  size_t depth = 0;


  bool chopRaw = false;
  int byteSize = 4;
  for (int i=1;i<ac;i++) {
    const std::string arg = av[i];
    if (arg[0] != '-')
        inFileName = arg;
    else if (arg == "-o")
        outFileBase = av[++i];
    else if (arg == "-v" || arg == "--variable")
        varName = av[++i];
    else if (arg == "-b" || arg == "--bytes")
        outputAsBytes = true;
    else if (arg == "-n" || arg == "--normalize")
        normalize = true;
    else if (arg == "-m" || arg == "--mask") {
      hasMask = true;
      maskFileName = av[++i];
    }
    else if (arg == "-cz" || arg == "--chop_z") {
      chopZ = true;
      startZ = std::stoi(av[++i]);
      endZ = std::stoi(av[++i]);
    }
    else if (arg == "-craw") {
      chopRaw = true;
      byteSize = std::stoi(av[++i]);
    }else if (arg == "-dims") {
      width = std::stoi(av[++i]);
      height = std::stoi(av[++i]);
      depth = std::stoi(av[++i]);
    }
    else throw std::runtime_error("unknown cmdline arg '"+arg+"'");
  }
  if (inFileName.empty())
    throw std::runtime_error("no input file name specified....");

  if(chopRaw && !chopZ)
  {
    throw std::runtime_error("Cant chop raw file without chopZ values");
  }

  if(chopRaw)
  {
    std::ifstream inRawFile(inFileName, std::ios::binary | std::ios::in);
    std::vector<float> fileContent(width*height*depth);
    inRawFile.read((char*)fileContent.data(), width*height*depth*byteSize);
    inRawFile.close();
    std::ofstream outRawFile(inFileName + "_chopped_out.raw", std::ios::binary);
    size_t startElement = startZ * width * height;
    size_t sizeToWrite = (endZ - startZ + 1) * width*height * byteSize;
    
    outRawFile.write((char*)&fileContent[startElement], sizeToWrite);

    return EXIT_SUCCESS;
  }



  int status = 0;
  fitsfile *dataFile;
   
  fits_open_file(&dataFile,inFileName.c_str(),READONLY,&status);

  if (status != 0) {
    fits_report_error(stderr,status);
    fits_close_file(dataFile,&status);
    return EXIT_FAILURE;
  }

  if (!checkFitsFile(dataFile)) {
      return EXIT_FAILURE;
  }

  int bitpix = 0;
  long naxes[3];
  int naxis = 3;
  fits_get_img_param(dataFile,3,&bitpix,&naxis,naxes,&status);

  if(chopZ)
  {
    if(startZ < 0 || startZ >= static_cast<size_t>(naxes[2]))
    {
      startZ = 0;
    }

    if(endZ < startZ || endZ >= static_cast<size_t>(naxes[2]))
    {
      endZ = static_cast<size_t>(naxes[2]);
    }
    
  } else
  {
    startZ = 0;
    endZ = naxes[2] - 1;
  }

  fprintf(stdout,"FITS img size is: (%i,%i,%i)\n",(int)naxes[0],(int)naxes[1],(int)naxes[2]);
  if (bitpix == BYTE_IMG) fprintf(stdout,"FITS data type is: BYTE\n");
  else if (bitpix == SHORT_IMG) fprintf(stdout,"FITS data type is: SHORT\n");
  else if (bitpix == LONG_IMG) fprintf(stdout,"FITS data type is: LONG\n");
  else if (bitpix == LONGLONG_IMG) fprintf(stdout,"FITS data type is: LONGLONG\n");
  else if (bitpix == FLOAT_IMG) fprintf(stdout,"FITS data type is: FLOAT\n");
  else if (bitpix == DOUBLE_IMG) fprintf(stdout,"FITS data type is: DOUBLE\n");

  if (bitpix != FLOAT_IMG) {
    fprintf(stderr,"FITS unsupported image format\n");
    fits_close_file(dataFile,&status);
    return EXIT_FAILURE;
  }

  fitsfile* maskFile=0;

  if (hasMask) {
  
      fits_open_file(&maskFile, maskFileName.c_str(), READONLY, &status);

      if (status != 0) {
          fits_report_error(stderr, status);
          fits_close_file(maskFile, &status);
          return EXIT_FAILURE;
      }

      if (!checkFitsFile(maskFile)) {
          return EXIT_FAILURE;
      }

      int bitMask = 0;
      long naxesMask[3];
      int naxisMask = 3;
      fits_get_img_param(maskFile, 3, &bitMask, &naxisMask, naxesMask, &status);
      if (bitMask != LONG_IMG || naxisMask!=3 || naxesMask[0] != naxes[0] || naxesMask[1] != naxes[1] || naxesMask[2] != naxes[2]) {
          fprintf(stderr, "FITS Sofia mask does not match to input file\n");
          return EXIT_FAILURE;
      }
  }


  if (outputAsBytes) {
      std::vector<char> values(naxes[0] * size_t(naxes[1]) * naxes[2]);

      long fpixel[3] = { 1,1,1 };
      int retVal = fits_read_pix(dataFile, TBYTE, fpixel, naxes[0] * size_t(naxes[1]) * naxes[2], 0,
          values.data(), 0, &status);
      fprintf(stdout, "FITS retVal: %i, status: %i\n", retVal, status);

      if (hasMask) {
          maskValues(values, maskFile, naxes);
      }

      std::string fileName = outFileBase + "_" + varName + "_"
          + std::to_string(naxes[0]) + "x"
          + std::to_string(naxes[1]) + "x"
          + std::to_string(naxes[2]) + "_bytes.raw";
      std::ofstream out(fileName, std::ios::binary);
      out.write((const char*)values.data(), values.size() * sizeof(values[0]));
      std::cout << "written to " << fileName << std::endl;
  }
  else {
    size_t vSIze = static_cast<size_t>(naxes[0])*naxes[1]*naxes[2];
    std::vector<float> values(vSIze);
    long fpixel[3] = {1,1,1};
    int retVal = fits_read_pix(dataFile,TFLOAT, fpixel, vSIze,0,
                             values.data(),0,&status);
    fprintf(stdout,"FITS retVal: %i, status: %i\n",retVal,status);

    if (normalize) {
        auto maxValue = *(std::max_element(std::begin(values), std::end(values))); // C++11
        auto minValue = *(std::min_element(std::begin(values), std::end(values))); // C++11
        std::cout << "Normalizing with Min: " << minValue << " Max: " << maxValue << "\n";

        /*
        float minScale=0.45f;
        float maxScale=0.1f;
        float scaleFactor = 1.f/maxScale;
        minScale = scaleFactor * minScale;
        */

        /*
        float minScale=0.012f;
        float maxScale=0.015f;
        float scaleFactor = 1.f/maxScale;
        minScale = scaleFactor * minScale;
        */
        
        float minScale = 0.0f;
        float maxScale = 0.1f;
        float scaleFactor = 1.f / maxScale;
        minScale = scaleFactor * minScale;

        std::transform(values.begin(), values.end(), values.begin(), [&](float v) {
            float normalizedValue = (v - minValue) / std::abs(maxValue - minValue);

            normalizedValue = (normalizedValue * scaleFactor) - minScale;

            normalizedValue = std::max(std::min(normalizedValue, 1.f), 0.f);
            //std::cout << "Input: " << v << " : " << normalizedValue << "\n";
            return normalizedValue;
            }  );
    }
  
    auto maxValue = *(std::max_element(std::begin(values), std::end(values))); // C++11
    auto minValue = *(std::min_element(std::begin(values), std::end(values))); // C++11
    std::cout << "Min: " << minValue << " Max: " << maxValue << "\n";

    if (hasMask) {
        maskValues(values, maskFile, naxes);
    }

    std::string fileName = outFileBase + "_" + varName + "_"
        + std::to_string(naxes[0]) + "x"
        + std::to_string(naxes[1]) + "x"
        + std::to_string(endZ - startZ + 1) + "_float.raw";
    size_t startElement = startZ * naxes[0] * naxes[1];
    size_t sizeToWrite = (endZ - startZ + 1) * naxes[0] * naxes[1] * sizeof(values[0]);
    std::ofstream out(fileName,std::ios::binary);
    out.write((const char *)&values[startElement],sizeToWrite);
    
    std::cout << "written to " << fileName << std::endl;
  }

  fits_close_file(dataFile,&status);
  return EXIT_SUCCESS;
}
