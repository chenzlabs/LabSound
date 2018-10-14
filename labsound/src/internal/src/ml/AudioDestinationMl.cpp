// License: BSD 3 Clause
// Copyright (C) 2010, Google Inc. All rights reserved.
// Copyright (C) 2015+, The LabSound Authors. All rights reserved.

#include "internal/ml/AudioDestinationMl.h"
#include "internal/VectorMath.h"

#include "LabSound/core/AudioNode.h"
#include "LabSound/core/AudioIOCallback.h"
#include "LabSound/extended/Logging.h"

namespace lab
{

const float kLowThreshold = -1.0f;
const float kHighThreshold = 1.0f;

void outputBufferCallback(MLHandle handle, void *callback_context) {
    AudioDestinationMl *audioDestination = (AudioDestinationMl *)callback_context;

    MLAudioBuffer outputMlBuffer;
    MLResult result = MLAudioGetOutputStreamBuffer(
      audioDestination->outputHandle,
      &outputMlBuffer 
    );
    if (result != MLResult_Ok) {
      std::cerr << "failed to get ml output buffer" << std::endl;
    }

    MLAudioBuffer inputMlBuffer;
    if (audioDestination->isRecording()) {
      result = MLAudioGetInputStreamBuffer(
        audioDestination->inputHandle,
        &inputMlBuffer
      );
      if (result != MLResult_Ok) {
        std::cerr << "failed to get ml output buffer" << std::endl;
      }

      uint8_t *dstData[] = {
        (uint8_t *)audioDestination->inputBuffer.data(),
      };
      int dstDataSize = audioDestination->inputBuffer.size();
      const uint8_t *srcData[] = {
        inputMlBuffer.ptr,
      };
      int srcDataSize = inputMlBuffer.size / sizeof(uint16_t);
      swr_convert(audioDestination->input_swr_ctx, dstData, dstDataSize, srcData, srcDataSize);
    } else {
      memset(audioDestination->inputBuffer.data(), 0, sizeof(float) * audioDestination->inputBuffer.size());
    }

    audioDestination->render(audioDestination->outputBuffer.size(), audioDestination->outputBuffer.data(), audioDestination->inputBuffer.data());
    
    uint8_t *dstData[] = {
      outputMlBuffer.ptr,
    };
    int dstDataSize = outputMlBuffer.size / 2 / sizeof(uint16_t);
    const uint8_t *srcData[] = {
      (const uint8_t *)audioDestination->outputBuffer.data(),
    };
    int srcDataSize = audioDestination->outputBuffer.size() / 2;
    swr_convert(audioDestination->output_swr_ctx, dstData, dstDataSize, srcData, srcDataSize);

    result = MLAudioReleaseOutputStreamBuffer(audioDestination->outputHandle);
    if (result != MLResult_Ok) {
      std::cerr << "failed to release ml output buffer" << std::endl;
    }

    if (audioDestination->isRecording()) {
      result = MLAudioReleaseInputStreamBuffer(audioDestination->inputHandle);
      if (result != MLResult_Ok) {
        std::cerr << "failed to release ml input buffer" << std::endl;
      }
    }
}

void inputBufferCallback(MLHandle handle, void *callback_context) {
  // nothing
}

AudioDestination * AudioDestination::MakePlatformAudioDestination(AudioIOCallback & callback, unsigned numberOfOutputChannels, float sampleRate)
{
    return new AudioDestinationMl(callback, sampleRate);
}

unsigned long AudioDestination::maxChannelCount()
{
    return 2;
}

AudioDestinationMl::AudioDestinationMl(AudioIOCallback & callback, float sampleRate) : m_callback(callback)
{
    outputAudioBufferFormat.bits_per_sample = 16;
    outputAudioBufferFormat.channel_count = 2;
    outputAudioBufferFormat.sample_format = MLAudioSampleFormat_Int;
    outputAudioBufferFormat.samples_per_second = (uint32_t)sampleRate;
    outputAudioBufferFormat.valid_bits_per_sample = 16;
    
    uint32_t nOutputBufferFrames = (uint32_t)sampleRate / 10;
    outputBuffer = std::vector<float>(nOutputBufferFrames * 2);
    
    {
      MLResult result = MLAudioCreateSoundWithOutputStream(
        &outputAudioBufferFormat,
        nOutputBufferFrames * 2 * sizeof(uint16_t),
        outputBufferCallback,
        this,
        &outputHandle
      );
      if (result != MLResult_Ok) {
        std::cerr << "failed to create ml output sound: " << result << std::endl;
      }
    }

    inputAudioBufferFormat.bits_per_sample = 16;
    inputAudioBufferFormat.channel_count = 1;
    inputAudioBufferFormat.sample_format = MLAudioSampleFormat_Int;
    inputAudioBufferFormat.samples_per_second = 16000;
    inputAudioBufferFormat.valid_bits_per_sample = 16;
    
    uint32_t nInputBufferFrames = nOutputBufferFrames * inputAudioBufferFormat.samples_per_second / outputAudioBufferFormat.samples_per_second;
    inputBuffer = std::vector<float>(nOutputBufferFrames);
    
    {
      MLResult result = MLAudioCreateInputFromVoiceComm(
        &inputAudioBufferFormat,
        nInputBufferFrames * sizeof(uint16_t),
        inputBufferCallback,
        nullptr,
        &inputHandle
      );
      if (result != MLResult_Ok) {
        std::cerr << "failed to create ml microphone input: " << result << std::endl;
      }
    }

    m_sampleRate = sampleRate;
    m_renderBus.setSampleRate(m_sampleRate);
    m_inputBus.setSampleRate(m_sampleRate);
    // m_inputBus.setSampleRate((float)inputAudioBufferFormat.samples_per_second);
    // configure();

    {
      int64_t src_ch_layout = AV_CH_LAYOUT_STEREO, dst_ch_layout = AV_CH_LAYOUT_STEREO;
      enum AVSampleFormat src_sample_fmt = AV_SAMPLE_FMT_FLT, dst_sample_fmt = AV_SAMPLE_FMT_S16;
      int src_rate = outputAudioBufferFormat.samples_per_second, dst_rate = outputAudioBufferFormat.samples_per_second;
      
      output_swr_ctx = swr_alloc_set_opts(
        nullptr,
        dst_ch_layout,
        dst_sample_fmt,
        dst_rate,
        src_ch_layout,
        src_sample_fmt,
        src_rate,
        0,
        nullptr
      );
      if (!output_swr_ctx) {
        std::cerr << "failed to allocate output resmapler context" << std::endl;
      }

      /* initialize the resampling context */
      if (swr_init(output_swr_ctx) < 0) {
        std::cerr << "failed to initialize output resampler context" << std::endl;
      }
    }
    
    {
      int64_t src_ch_layout = AV_CH_LAYOUT_MONO, dst_ch_layout = AV_CH_LAYOUT_MONO;
      enum AVSampleFormat src_sample_fmt = AV_SAMPLE_FMT_S16, dst_sample_fmt = AV_SAMPLE_FMT_FLT;
      int src_rate = inputAudioBufferFormat.samples_per_second, dst_rate = outputAudioBufferFormat.samples_per_second;
      
      input_swr_ctx = swr_alloc_set_opts(
        nullptr,
        dst_ch_layout,
        dst_sample_fmt,
        dst_rate,
        src_ch_layout,
        src_sample_fmt,
        src_rate,
        0,
        nullptr
      );
      if (!input_swr_ctx) {
        std::cerr << "failed to allocate input resmapler context" << std::endl;
      }

      /* initialize the resampling context */
      if (swr_init(input_swr_ctx) < 0) {
        std::cerr << "failed to initialize input resampler context" << std::endl;
      }
    }
}

AudioDestinationMl::~AudioDestinationMl()
{
  swr_free(&output_swr_ctx);
  swr_free(&input_swr_ctx);
    // dac.release(); // XXX
    /* if (dac.isStreamOpen())
        dac.closeStream(); */
}

/* void AudioDestinationMl::configure()
{
    if (dac->getDeviceCount() < 1)
    {
        LOG_ERROR("No audio devices available");
    }

    dac->showWarnings(true);

    RtAudio::StreamParameters outputParams;
    outputParams.deviceId = dac->getDefaultOutputDevice();
    outputParams.nChannels = 2;
    outputParams.firstChannel = 0;

	auto deviceInfo = dac->getDeviceInfo(outputParams.deviceId);
	LOG("Using Default Audio Device: %s", deviceInfo.name.c_str());

    RtAudio::StreamParameters inputParams;
    inputParams.deviceId = dac->getDefaultInputDevice();
    inputParams.nChannels = 1;
    inputParams.firstChannel = 0;

    unsigned int bufferFrames = AudioNode::ProcessingSizeInFrames;

    RtAudio::StreamOptions options;
    options.flags |= RTAUDIO_NONINTERLEAVED;

    try
    {
        dac->openStream(&outputParams, &inputParams, RTAUDIO_FLOAT32, (unsigned int) m_sampleRate, &bufferFrames, &outputCallback, this, &options);
    }
    catch (RtAudioError & e)
    {
        e.printMessage();
    }
} */

void AudioDestinationMl::start()
{
    MLResult result = MLAudioStartSound(outputHandle);

    if (result == MLResult_Ok) {
      m_isPlaying = true;
    } else {
      std::cerr << "failed to start ml output sound" << std::endl;
    }
}

void AudioDestinationMl::stop()
{
    MLResult result = MLAudioStopSound(outputHandle);

    if (result == MLResult_Ok) {
      m_isPlaying = false;
    } else {
      std::cerr << "failed to stop ml output sound" << std::endl;
    }
}

void AudioDestinationMl::startRecording()
{
    MLResult result = MLAudioStartSound(inputHandle);

    if (result == MLResult_Ok) {
      m_isRecording = true;
    } else {
      std::cerr << "failed to start ml input" << std::endl;
    }
}

void AudioDestinationMl::stopRecording()
{
    MLResult result = MLAudioStopSound(inputHandle);

    if (result == MLResult_Ok) {
      m_isRecording = false;
    } else {
      std::cerr << "failed to stop ml input" << std::endl;
    }
}

// Pulls on our provider to get rendered audio stream.
void AudioDestinationMl::render(int numberOfFrames, void *outputBuffer, void *inputBuffer)
{
    // Inform bus to use an externally allocated buffer from rtaudio
    m_renderBus.setChannelMemory(0, (float *)outputBuffer, numberOfFrames);
    m_renderBus.setChannelMemory(1, (float *)outputBuffer + numberOfFrames, numberOfFrames);

    m_inputBus.setChannelMemory(0, (float *)inputBuffer, numberOfFrames);

    // Source Bus :: Destination Bus 
    m_callback.render(&m_inputBus, &m_renderBus, numberOfFrames);

    // Clamp values at 0db (i.e., [-1.0, 1.0])
    for (unsigned i = 0; i < m_renderBus.numberOfChannels(); ++i)
    {
        AudioChannel * channel = m_renderBus.channel(i);
        VectorMath::vclip(channel->data(), 1, &kLowThreshold, &kHighThreshold, channel->mutableData(), 1, numberOfFrames);
    }
}

} // namespace lab
