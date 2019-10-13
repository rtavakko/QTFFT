#include "audioprocessor.h"

audioprocessor::audioprocessor()
{
    initAudio();
    initQuantizer();

    start(QThread::LowPriority);
}

audioprocessor::~audioprocessor()
{
    timer->stop();
    ioDevice->close();

    delete timer;
    timer = nullptr;

    delete inputDevice;
    inputDevice = nullptr;

    delete ioDevice;
    ioDevice = nullptr;

    delete spectrumRangeArrayBins;
    spectrumRangeArrayBins = nullptr;
}

void audioprocessor::initAudio()
{
    //Set audio format
    format.setSampleRate(44100);
    format.setChannelCount(1);
    format.setSampleSize(8);
    format.setByteOrder(QAudioFormat::LittleEndian);
    format.setCodec("audio/pcm");

    inputDevice = nullptr;
    ioDevice = nullptr;

    //Get current default audio input device, compare with set format and revise format based on device specifications
    QAudioDeviceInfo inputDeviceInfo(QAudioDeviceInfo::defaultInputDevice());
    if(!inputDeviceInfo.isFormatSupported(format))
    {
        format = inputDeviceInfo.nearestFormat(format);
    }

    if(format.isValid())
    {
        inputDevice = new QAudioInput(inputDeviceInfo,format,this);

        ioDevice = inputDevice->start();
        inputDevice->setBufferSize(inputDevice -> periodSize());    //Set number of audio samples provided by buffer
        inputDevice->setNotifyInterval(inputDevice -> periodSize());    //Set notification period for audio device

        ioDevice->open(QIODevice::ReadOnly);
    }

    windowSize = inputDevice->bufferSize();
    bins = windowSize/2;

    fmax = format.sampleRate()/2.0;
    samplePeriod = 1000.0*static_cast<double>(windowSize) / static_cast<double>(format.sampleRate());
    freqResolution = fmax / bins;

    dataMinValue = 0;
    dataMaxValue = 0;

    aveSpectrumSize = bins/2;

    spectrumRangeArrayBins = new int[spectrumRangeArrayNumBins];
    beatDetectThreshold = 0.6;

    QObject::connect(ioDevice,SIGNAL(aboutToClose()),this,SLOT(updateState()));
    QObject::connect(ioDevice,SIGNAL(readyRead()),this,SLOT(process()));    //Call to process data when buffer is ready
}

void audioprocessor::initQuantizer()
{
    quantizePeriod = 10;
    quantizeFlag = false;

    timer = new QTimer();

    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(quantize()));
    timer->start(quantizePeriod);
}

bool audioprocessor::updateRangeArray(int* rangeArray, int numFrequencies)
{
    if(rangeArray != nullptr && numFrequencies == spectrumRangeArrayNumBins)
    {
        for(int i = 0; i < spectrumRangeArrayNumBins; i++)
        {
            spectrumRangeArrayBins[i] = rangeArray[i];
        }
        return true;
    }
    else
    {
        return false;
    }
}

void audioprocessor::resizeDataBuffer()
{
    int newSize = inputDevice->periodSize();    //Get current size

    newSize = static_cast<int> (ceil(log(newSize)/log(2))); //Find power of 2 for current size and round to nearest integer
    newSize = pow(2,newSize); //Recalculate size

    dataBuffer.resize(newSize,0);   //Resize data buffer and zero-pad

    //Update FFT variables accordingly
    windowSize = dataBuffer.size();
    bins = windowSize/2;

    fmax = format.sampleRate()/2.0;
    samplePeriod = 1000.0*static_cast<double>(windowSize) / static_cast<double>(format.sampleRate());
    freqResolution = fmax / bins;
}

void audioprocessor::readAudioData()
{
    //Written for 8-bit audio; TODO: revise for other bitrates
    int channelBytes = format.sampleSize() / 8; //Numer of bytes per data point (e.g. 2xbytes for 16-bit audio)
    int totalBytes = channelBytes * format.channelCount();  //Number of channels (e.g. 16-bit stereo audio: 2 channels x 2 bytes -> 4 bytes per stereo sample)

    QByteArray buffer = ioDevice -> read(static_cast<qint64>(inputDevice->periodSize()));   //Read audio bytes
    const char* rawData = buffer.constData();   //Get pointer to the hardware data array

    qint8 valueStereo = 0; //Value of each of the 2 channels if using stereo audio
    qint8 valueMono = 0;   //The bigger value of both channels if using stereo audio

    qint8 valueMinLimit = 0;  //Minimum data value calculated based on specifications
    qint8 valueMaxLimit = 0;  //Maximum data value calculated based on specifications

    dataBuffer.clear(); //Clear previous values in data buffer

    //Traverse the values stored in interleaved format ( [1][L,R],[2][L,R]...)
    for(int i = 0; i < buffer.size(); i+= totalBytes)
    {
        for(int j = 0; j < channelBytes; j++)   //  [n][L,R]
        {
            //Map signed values to [0-valueLimit] since the data buffer stores unsigned values
            if(format.sampleSize() == 8 && format.sampleType() == QAudioFormat::UnSignedInt)
            {
                //[0,(2^8)-1]
                if(format.byteOrder() == QAudioFormat::BigEndian)
                {
                    valueStereo = qFromBigEndian<quint8>(*rawData);
                }
                else
                {
                    valueStereo = qFromLittleEndian<quint8>(*rawData);
                }
            }
            else if(format.sampleSize() == 8 && format.sampleType() == QAudioFormat::SignedInt)
            {
                //qDebug() << "HERE";
                //[-(2^7),(2^7)-1] -> [0,255]
                valueMinLimit = pow(2,format.sampleSize()-1);
                if(format.byteOrder() == QAudioFormat::BigEndian)
                {
                    valueStereo = qFromBigEndian<qint8>(*rawData);
                    valueStereo += valueMinLimit;
                }
                else
                {
                    valueStereo = qFromLittleEndian<qint8>(*rawData);
                    valueStereo += valueMinLimit;
                }
            }
            else if(format.sampleSize() == 16 && format.sampleType() == QAudioFormat::UnSignedInt)
            {
                //[0,(2^16)-1]
                valueMinLimit = pow(2,format.sampleSize())-1;
                if(format.byteOrder() == QAudioFormat::BigEndian)
                {
                    valueStereo = qFromBigEndian<quint16>(*rawData);
                }
                else
                {
                    valueStereo = qFromLittleEndian<quint16>(*rawData);
                }
            }
            else if(format.sampleSize() == 16 && format.sampleType() == QAudioFormat::SignedInt)
            {
                //[-(2^15),(2^15)-1] -> [0,65535]
                valueMinLimit = pow(2,format.sampleSize()-1);
                if(format.byteOrder() == QAudioFormat::BigEndian)
                {
                    valueStereo = qFromBigEndian<qint16>(*rawData);
                    valueStereo += valueMinLimit;
                }
                else
                {
                    valueStereo = qFromLittleEndian<qint16>(*rawData);
                    valueStereo += valueMinLimit;
                }
            }
            else if(format.sampleSize() == 32 && format.sampleType() == QAudioFormat::UnSignedInt)
            {
                //[0,(2^32)-1]
                valueMinLimit = pow(2,format.sampleSize()) - 1;
                if(format.byteOrder() == QAudioFormat::BigEndian)
                {
                    valueStereo = qFromBigEndian<quint32>(*rawData);
                }
                else
                {
                    valueStereo = qFromLittleEndian<quint32>(*rawData);
                }
            }
            else if(format.sampleSize() == 32 && format.sampleType() == QAudioFormat::SignedInt)
            {
                //[-(2^31),(2^31)-1] -> [0,(2^32)-1]
                valueMinLimit = pow(2,format.sampleSize()-1);
                if(format.byteOrder() == QAudioFormat::BigEndian)
                {
                    valueStereo = qFromBigEndian<qint32>(*rawData);
                    valueStereo += valueMinLimit;
                }
                else
                {
                    valueStereo = qFromLittleEndian<qint32>(*rawData);
                    valueStereo += valueMinLimit;
                }
            }
            valueMono = (valueStereo > valueMono)?valueStereo:valueMono;    //Take the higher value from the 2 channels if using stereo audio

            rawData += channelBytes;    //Move pointer to the next channel data to traverse the audio buffer
        }
        dataBuffer.push_back(valueMono);

        //Set value limits to [0-valueLimit]
        dataMinValue = 0;
        dataMaxValue = valueMinLimit;

        //Reset values
        valueStereo = 0;
        valueMono = 0;
        valueMinLimit = 0;
    }
}

void audioprocessor::readFileData(std::string inputFile)
{
    dataBuffer.clear();

    file.setFileName(QDir::currentPath() + QString::fromStdString(inputFile));
    QByteArray line;
    quint8 rawValue;

    if(file.open(QIODevice::ReadOnly))
    {
        while(!file.atEnd())
        {
            line = file.readLine();
            line = line.split(',').first();
            rawValue = line.toInt();
            dataBuffer.push_back(static_cast<quint8>(rawValue));
        }
    }

    //Resize data buffer to the nearest power of 2
    int newSize = dataBuffer.size();

    newSize = static_cast<int> (ceil(log(newSize)/log(2)));
    newSize = pow(2,newSize);

    dataBuffer.resize(newSize,0);

    //Update FFT variables accordingly
    windowSize = dataBuffer.size();
    bins = windowSize/2;

    fmax = format.sampleRate()/2.0;
    samplePeriod = 1000.0*static_cast<double>(windowSize) / static_cast<double>(format.sampleRate());
    freqResolution = fmax / bins;

    file.close();
}

void audioprocessor::writeFileData()
{
    //Output format: "time-domain value,FFT(RE),FFT(IM),CR-LF"
    QByteArray tmp;
    QString str;

    if(file.open(QFile::WriteOnly))
    {
        QTextStream out(&file);
        for(int i = 0; i < spectrumOutput.size(); i++)
        {
            out << dataBuffer[i] << "," << spectrumOutput[i].real() << "," << spectrumOutput[i].imag() << "\r\n";
        }
        //out << "Complexity: O(" << complexity << ")";
        //out << "ELAPSED TIME:" << chrono::duration_cast<chrono::nanoseconds>(end-start).count() << " nS";
        //out << "ELAPSED TIME:" << chrono::duration_cast<chrono::microseconds>(end-start).count() << " uS";
        //out << "ELAPSED TIME:" << chrono::duration_cast<chrono::milliseconds>(end-start).count() << " mS";
    }
    file.close();
}

void audioprocessor::getFFT(int nSamples)
{
    //Find FFT results and processing time
    spectrumOutput.clear();
    spectrumOutput.resize(nSamples,0);

    auto start = std::chrono::steady_clock::now();

    complexity = 0;

    spectrumOutput = getFFTBins(0,1,dataBuffer.size()); //In-place alternative: getFFTBinsInPlace(0,1,dataBuffer.size());

    auto end = std::chrono::steady_clock::now();

    aveSpectrumOutput.clear();

    //Find average FFT results
    getFFTBinsAve();
    emit(sendAudioSpectrum(aveSpectrumOutput,dataMinValue,dataMaxValue));

    //If a quantize flag is set, find specified frequency FFTs
    if(quantizeFlag)
    {
        getFFTBinsArray();
        quantizeFlag = false;
        emit(sendAudioArray(spectrumRangeArray,dataMinValue,dataMaxValue,beatDetected));
    }

    //qDebug() << "Complexity: O(" << complexity << ")";
    //qDebug() << "ELAPSED TIME:" << chrono::duration_cast<chrono::nanoseconds>(end-start).count() << " nS";
    //qDebug() << "ELAPSED TIME:" << chrono::duration_cast<chrono::microseconds>(end-start).count() << " uS";
    //qDebug() << "ELAPSED TIME:" << chrono::duration_cast<chrono::milliseconds>(end-start).count() << " mS";
}

std::vector<std::complex<double>> audioprocessor::getFFTBins(int startIndex, int steps, int n)
{
    std::complex<double> numberI(0,1);

    std::vector<std::complex<double>> FFTOut(n,0);

    std::vector<std::complex<double>> FFTEven(n/2,0);
    std::vector<std::complex<double>> FFTOdd(n/2,0);

    std::complex<double> XA, XB;

    //Base case
    if(n == 1)
    {
        complexity++;
        FFTOut[0] = static_cast<std::complex<double>>(dataBuffer[startIndex]);
        return FFTOut;
    }
    else
    {
        FFTEven = getFFTBins(startIndex,steps*2,n/2);   //Find even FFT; start at current index and 2x current stepsize
        FFTOdd = getFFTBins(startIndex+steps,steps*2,n/2);  //Find odd FFT; start at current index with offset equal to stepsize and 2x current stepsize

        //Get FFT results
        for(int k = 0; k < n/2; k++)
        {
            XA = FFTEven[k] + exp(-numberI*2.0*M_PI*double(k)/double(n))*FFTOdd[k];
            XB = FFTEven[k] - exp(-numberI*2.0*M_PI*double(k)/double(n))*FFTOdd[k];

            //Store values for first and second halves of the buffer at the same time (XA and XB are complex conjugates)
            FFTOut[k] = XA;
            FFTOut[k+n/2] = XB;

            complexity++;
        }
        return FFTOut;
    }
}

void audioprocessor::getFFTBinsInPlace(int startIndex, int steps, int n)
{
    std::complex<double> numberI(0,1);

    std::complex<double> FFTEven(0,0);
    std::complex<double> FFTOdd(0,0);

    std::complex<double> XA, XB;

    int evenIndex = startIndex;
    int oddIndex = startIndex + steps;

    int sortIndex = 0;
    int endIndex = 0;

    if(n == 1)
    {
        complexity++;

        spectrumOutput[startIndex] = static_cast<std::complex<double>>(dataBuffer[startIndex]);
    }
    else
    {
        getFFTBinsInPlace(startIndex,steps*2,n/2);  //Evens
        getFFTBinsInPlace(startIndex+steps,steps*2,n/2);    //Odds

        for(int k = 0; k < n/2; k++)
        {
            complexity++;

            XA = spectrumOutput[evenIndex]+ exp(-numberI*2.0*M_PI*double(k)/double(n))*spectrumOutput[oddIndex];
            XB = spectrumOutput[evenIndex] - exp(-numberI*2.0*M_PI*double(k)/double(n))*spectrumOutput[oddIndex];

            spectrumOutput[evenIndex] = XA;
            spectrumOutput[oddIndex] = XB;

            evenIndex += 2*steps;
            oddIndex += 2*steps;
        }

        evenIndex = startIndex;
        oddIndex = startIndex + steps;

        sortIndex = oddIndex;
        endIndex = (n-1)*steps + evenIndex - steps;

        while(sortIndex < evenIndex + steps*n/2)
        {
            for(int i = sortIndex; i < endIndex; i+=2*steps)
            {
                complexity++;

                XA = spectrumOutput[i];
                XB = spectrumOutput[i+steps];

                spectrumOutput[i] = XB;
                spectrumOutput[i+steps] = XA;
            }
            sortIndex +=steps;
            endIndex -=steps;
        }

        //WIP in-place algorithm
        /*
        sortIndex = oddIndex;

        for(int i = sortIndex; i < evenIndex + steps*n/2; i+= steps*2)
        {
            sortIndex = i;
            endIndex = -1;
            XB = spectrumOutput[sortIndex];

            qDebug() << "STARTING HOP AT:" << i;

            while(true)
            {
                XA = spectrumOutput[sortIndex];
                spectrumOutput[sortIndex] = XB;
                XB = XA;

                if(endIndex == i)
                {
                    qDebug() << "ENDING HOP AT:" << endIndex;
                    break;
                }
                else if(sortIndex < evenIndex + steps*n/2)
                {
                    complexity++;
                    if((sortIndex - oddIndex) % (2*steps) == 0)
                    {
                        endIndex = ((sortIndex-oddIndex)/(2*steps))*steps + evenIndex + steps*n/2;
                    }
                    else
                    {
                        endIndex = sortIndex - steps*((sortIndex-evenIndex)/(2*steps));
                    }
                }
                else
                {
                    complexity++;
                    if((((n - 1)*steps +evenIndex - steps) - sortIndex) % (2*steps) == 0)
                    {
                        endIndex = steps*n/2  + evenIndex - steps - (((n-1)*steps + evenIndex -steps-(sortIndex))/(2*steps))*steps;
                        qDebug() << "INFO:" << (((n-1)*steps-steps-(sortIndex))/(2*steps))*steps;
                    }
                    else
                    {
                        endIndex = sortIndex + (((n-1)*steps + evenIndex - sortIndex)/(2*steps))*steps;
                    }
                }

                qDebug() << "Hopping from " << sortIndex << "to " << endIndex;

                sortIndex = endIndex;
            }
        }*/
    }
}

void audioprocessor::getFFTBinsAve()
{
    //Find a specified number of average values using the first half of the FFT data (second half is a mirrored duplicate)
    std::complex<double> currValue;

    double aveSpectrumValue = 0;
    int aveSpectrumStep = static_cast<int> (ceil((spectrumOutput.size()/2.0) / aveSpectrumSize));   //Calculate number of steps taken

    int count = 0;

    //Traverse first half of FFT results
    for(int i = 0; i < spectrumOutput.size()/2; i++)
    {
       currValue = static_cast<std::complex<double>>(2.0*spectrumOutput[i] / (double(spectrumOutput.size())));

        if(((count+1) % aveSpectrumStep) == 0 && count != 0)
        {
            aveSpectrumValue += sqrt(pow(currValue.real(),2.0)+pow(currValue.imag(),2.0));

            aveSpectrumValue = static_cast<double> (aveSpectrumValue / aveSpectrumStep);
            aveSpectrumOutput.push_back(aveSpectrumValue);

            aveSpectrumValue = 0;
            count = 0;
        }
        else
        {
            aveSpectrumValue += sqrt(pow(currValue.real(),2.0)+pow(currValue.imag(),2.0));

            count++;
        }
    }
}

void audioprocessor::getFFTBinsArray()
{
    //Designed for 3 ranges specified by 6 frequency values; TODO: expand for more ranges
    spectrumRangeArray.clear();

    double bassLevel = 0;
    double midLevel = 0;
    double treLevel = 0;

    bassLevel = getFFTFreqBin(spectrumRangeArrayBins[0],spectrumRangeArrayBins[1]).real();
    midLevel = getFFTFreqBin(spectrumRangeArrayBins[2],spectrumRangeArrayBins[3]).real();
    treLevel = getFFTFreqBin(spectrumRangeArrayBins[4],spectrumRangeArrayBins[5]).real();

    spectrumRangeArray.push_back(bassLevel);
    spectrumRangeArray.push_back(midLevel);
    spectrumRangeArray.push_back(treLevel);

    beatDetect();

    spectrumRangeArrayPrevious.clear();
    spectrumRangeArrayPrevious.assign(spectrumRangeArray.begin(),spectrumRangeArray.end());
}

std::complex<double> audioprocessor::getFFTFreqBin(double freq)
{
    int freqIndex = 0;

    freqIndex = static_cast<int> (floor(freq / freqResolution));

    if((freqIndex < 0) || (freqIndex > spectrumOutput.size() - 1))
    {
        return 0;
    }
    else
    {
        //Find magnitude of FFT bin. Normalize by dividing by size of FFT. Multiply by 2 to adjust amplitude since we discarded the mirrored seconds half
        return 2*sqrt(pow(spectrumOutput[freqIndex].real(),2.0)+pow(spectrumOutput[freqIndex].imag(),2.0))/(double(spectrumOutput.size()));
    }
}

std::complex<double> audioprocessor::getFFTFreqBin(double startFreq, double endFreq)
{
    std::complex<double> aveValue(0,0);

    int freqStartIndex = 0;
    int freqEndIndex = 0;

    freqStartIndex = static_cast<int> (floor(startFreq / freqResolution)); //Round down start frequency index
    freqEndIndex = static_cast<int> (ceil(endFreq / freqResolution));   //Round up end frequency index

    //qDebug() << "[NOTIFY PERIOD:" << inputDevice->notifyInterval() << "]";
    //qDebug() << "[FMAX:" << fmax << "," << "FRQRES:" << freqResolution << "]";
    //qDebug() << "[DATA SIZE:" << dataBuffer.size() << "," << "BUFFER SIZE:" << inputDevice->bufferSize() << "]";
    //qDebug() << "[" << startFreq  << "," << endFreq << "]->[" << freqStartIndex*freqResolution << "," << freqEndIndex*freqResolution << "]";
    //qDebug() << "-----------------------------------------------------------------------";
    if((freqStartIndex > freqEndIndex) || (freqStartIndex == freqEndIndex) || (freqEndIndex > spectrumOutput.size() - 1))
    {
        return 0;
    }
    else
    {
        for(int i = freqStartIndex; i < freqEndIndex; i++)
        {
            aveValue += spectrumOutput[i];
        }
        aveValue /= (freqEndIndex - freqStartIndex);    //Get average

        aveValue /= (double(spectrumOutput.size()));    //Normalize value
        aveValue *= 2;  //Adjust amplitude

        return sqrt(pow(aveValue.real(),2.0)+pow(aveValue.imag(),2.0)); //Return magnitude
    }
}

void audioprocessor::beatDetect()
{
    //Designed for 3 ranges specified by 6 frequency values; TODO: expand for more ranges
    beatDetected.clear();

    int numRanges = static_cast<int>(spectrumRangeArrayNumBins/2);

    if(!spectrumRangeArrayPrevious.empty()) //Make sure buffer from last calculation is filled
    {
        for(int i = 0; i < numRanges; i++)
        {
            double averageValue = (spectrumRangeArray[i] + spectrumRangeArrayPrevious[i]) / 2.0;  //Find average between the last calculation and this one
            if(spectrumRangeArray[i] > (1.0 + beatDetectThreshold)*averageValue)
            {
                beatDetected.push_back(true);   //If current value is higher than the average by the threshold, we have detected a beat in this range
            }
            else
            {
                beatDetected.push_back(false);
            }
        }
    }
}

unsigned int audioprocessor::bitReverse(unsigned int n)
{
    //Reverse bits of a binary number
    unsigned int num = n;
    unsigned int rev = 0;

    while(num > 0)
    {
        rev <<= 1;  //Left shift by 1 bit adding 0 at the end (adds 1 if value is 0)
        if((num & 1) == 1)  //If rightmost value is 1
        {
            rev ^= 1;   //Keep reverse bits as they are unless the number's bits are 1
        }
        num >>= 1; //Right shift by 1 bit adding 0 at the end
    }
    return rev;
}

void audioprocessor::run()
{
}

void audioprocessor::process()
{
    readAudioData();    //Read raw data
    resizeDataBuffer(); //Resize buffer and zero-pad
    getFFT(dataBuffer.size());  //Get FFT
    wait(); //Sleep until next cycle
}

void audioprocessor::quantize()
{
    quantizeFlag = true;
}

void audioprocessor::updateState()
{
}
