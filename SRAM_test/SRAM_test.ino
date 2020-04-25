/*

	Example of use of the FFT libray
        Copyright (C) 2014 Enrique Condes

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/*
  In this example, the Arduino simulates the sampling of a sinusoidal 1000 Hz
  signal with an amplitude of 100, sampled at 5000 Hz. Samples are stored
  inside the vReal array. The samples are windowed according to Hamming
  function. The FFT is computed using the windowed samples. Then the magnitudes
  of each of the frequencies that compose the signal are calculated. Finally,
  the frequency with the highest peak is obtained, being that the main frequency
  present in the signal.
*/

#include <SPI.h>
#include <serialram.h>
#include <virtmem.h>
#include <alloc/spiram_alloc.h>
#include "SRAM_FFT.h"

// configuration of SRAM chip: a 23LC1024 chip is assumed here which has CS connected to pin 9
const int chipSelect = 10;
const int chipSize = 1024 * 128; // 128 kB (=1 mbit)
const bool largeAddressing = true; // false if chipsize <1 mbit
const SerialRam::ESPISpeed spiSpeed = SerialRam::SPEED_FULL;

// pull in complete virtmem namespace
using namespace virtmem;

SPIRAMVAlloc valloc(chipSize, largeAddressing, chipSelect, spiSpeed);


arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 2048; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;


/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
// struct of virtual pointers
/*
struct dataStruct {
    VPtr<double, SPIRAMVAlloc> vpReal;
    VPtr<double, SPIRAMVAlloc> vpImag;
};
*/


#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  Serial.begin(115200);
  Serial.println("Ready");

  valloc.start();

  delay(3000); // add some delay so the user can connect with a serial terminal

}

void loop()
{

    VPtr<double, SPIRAMVAlloc> vpReal = valloc.alloc<double>(samples * 2 * sizeof(double));  //allocate memory for virtual data
    VPtr<double, SPIRAMVAlloc> vpImag = vpReal + samples + 1;

    /* Build raw data */
  double cycles = (((samples-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
  for (uint16_t i = 0; i < samples; i++)
  {
      // virtual pointers
      double y = (amplitude * (sin((i * (twoPi * cycles)) / samples))) / 2.0;
      vpReal[i] = y;/* Build data with positive and negative values*/
      vpImag[i] = (double)0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }



 //   VPtr<double, SPIRAMVAlloc> vpRealData = vpData->vpReal;
 //   VPtr<double, SPIRAMVAlloc> vpImagData = vpData->vpImag;

  // NOW FOR VIRTUAL POINTERS

    /* Print the results of the simulated sampling according to time */
    Serial.println("Data:");
    PrintVector(vpReal, samples, SCL_TIME);
    unsigned long now = millis();
    FFT.Windowing(vpReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    Serial.print("Weighed data time: "); Serial.println(millis() - now);
    now = millis();
    PrintVector(vpReal, samples, SCL_TIME);
    FFT.Compute(vpReal, vpImag, samples, FFT_FORWARD); /* Compute FFT */
    Serial.print("Computed Real values: "); Serial.println(millis() - now);
    PrintVector(vpReal, samples, SCL_INDEX);
    Serial.println("Computed Imaginary values:");
    PrintVector(vpImag, samples, SCL_INDEX);
    now = millis();
    FFT.ComplexToMagnitude(vpReal, vpImag, samples); /* Compute magnitudes */
    Serial.print("Computed magnitudes:"); Serial.println(millis() - now);
    PrintVector(vpReal, (samples >> 1), SCL_FREQUENCY);
    double x = FFT.MajorPeak(vpReal, samples, samplingFrequency);
    Serial.print("Major peak: "); Serial.println(millis() - now);
    Serial.println(x, 6);






    while(1); /* Run Once */
  // delay(2000); /* Repeat after delay */
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}


void PrintVector(VPtr<double, SPIRAMVAlloc> &vData, uint16_t bufferSize, uint8_t scaleType)
{
    for (uint16_t i = 0; i < bufferSize; i++)
    {
        double abscissa;
        /* Print abscissa value */
        switch (scaleType)
        {
            case SCL_INDEX:
                abscissa = (i * 1.0);
                break;
            case SCL_TIME:
                abscissa = ((i * 1.0) / samplingFrequency);
                break;
            case SCL_FREQUENCY:
                abscissa = ((i * 1.0 * samplingFrequency) / samples);
                break;
        }
        Serial.print(abscissa, 6);
        if(scaleType==SCL_FREQUENCY)
            Serial.print("Hz");
        Serial.print(" ");
        Serial.println(vData[i], 4);
    }
    Serial.println();
}

