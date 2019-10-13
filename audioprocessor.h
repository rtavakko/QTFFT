#ifndef AUDIOPROCESSOR_H
#define AUDIOPROCESSOR_H

#include <QDebug>
#include <QObject>

#include <QAudioInput>
#include <QAudioOutput>
#include <QIODevice>
#include <QAudioDeviceInfo>
#include <QByteArray>
#include <QtEndian>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QThread>
#include <QTimer>

#include <complex>
#include <math.h>
#include <chrono>

class audioprocessor : public QThread
{
    Q_OBJECT

private:

    QFile file; //File to read and write data to

    QAudioFormat format;    //Audio format device
    QAudioInput* inputDevice;   //Audio input device
    QIODevice* ioDevice;    //IO device to read from audio input device

    //FFT variables    
    int windowSize; //Total number of FFT samples (set to hardware buffer size)
    int bins;   //Number of usable (unmirrored) FFT samples (half of windowSize)

    double fmax;    //Maximum detectable frequency (e.g. 22000)
    double samplePeriod; //Length of time corresponding to the number of FFT samples (mS)
    double freqResolution;

    int complexity; //Algorithm cost estimate

    //Input audio data buffer and limits
    std::vector<qint8> dataBuffer;

    quint8 dataMaxValue;
    quint8 dataMinValue;

    //FFT results
    std::vector<std::complex<double>> spectrumOutput;

    //Averaged FFT results
    int aveSpectrumSize;
    std::vector<double> aveSpectrumOutput;

    //Specific frequency bin definitions
    static const int spectrumRangeArrayNumBins = 6; //This will need to be even
    int* spectrumRangeArrayBins;

    std::vector<double> spectrumRangeArray; //Stores FFT results for specific frequency ranges
    std::vector<double> spectrumRangeArrayPrevious; //Stores FFT results for specific frequency ranges from the last calculation for beat detection algorithm (spectogram concept)

    std::vector<bool> beatDetected;    //Stores booleans indicatig if a beat was detected in the specified frequency ranges
    double beatDetectThreshold; //Threshold that triggers beat detect flag if exceeded [0-1]

    //Timer used to quantize results further
    int quantizePeriod;
    bool quantizeFlag;

    QTimer* timer;

public:

    audioprocessor();
    ~audioprocessor();

    void initAudio();   //Initialize audio devices and variables
    void initQuantizer();   //Initialize quantization timer and variables

    bool updateRangeArray(int* rangeArray, int numFrequencies); //Updates spectrumRangeArrayBins

    void resizeDataBuffer();    //Resize the audio data buffer to the nearest rounded up power of 2
    void readAudioData();   //Read audio data into buffer based on format specifications
    void readFileData(std::string inputFile);   //Read data into buffer from file
    void writeFileData();   //Write data back to file

    void getFFT(int nSamples);  //Finds FFT results using either the regular or in-place algorithm

    //Cooley-Tukey FFT algorithm: regular and in-place
    std::vector<std::complex<double>> getFFTBins(int startIndex, int steps, int n);
    void getFFTBinsInPlace(int startIndex, int steps, int n);

    void getFFTBinsAve();   //Finds averages FFT values as specified by averaging variables
    void getFFTBinsArray(); //Populates spectrumRangeArray for specified frequency ranges

    std::complex<double> getFFTFreqBin(double freq);    //Find FFT result at frequency closest to specified value
    std::complex<double> getFFTFreqBin(double startFreq, double endFreq);   //Find FFT result average between two given frequencies

    void beatDetect();

    unsigned int bitReverse(unsigned int n);

signals:

    void sendAudioSpectrum(const std::vector<double> &data, double min, double max);
    void sendAudioArray(const std::vector<double> &data, double min, double max, const std::vector<bool> &beatDetectData);

protected:

    void run();

private slots:

    void process();
    void quantize();

    void updateState();

};

#endif
