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
const uint32_t mlBufferSize = 4096;

void processBuffers(AudioDestinationMl *audioDestination) {
  for (;;) {
    if (audioDestination->outputMlBuffers.size() > 0) {
      MLAudioBuffer &outputMlBuffer = audioDestination->outputMlBuffers.front();
      int totalOutputFrames = outputMlBuffer.size / 2 / sizeof(int16_t);
      int remainingOutputFrames = totalOutputFrames - audioDestination->outputIndex;
      int currentFrames = std::min<int>(remainingOutputFrames, AudioNode::ProcessingSizeInFrames);

      if (audioDestination->isRecording()) {
        if (audioDestination->inputMlBuffers.size() > 0) {
          std::vector<float> &inputBuffer = audioDestination->inputBuffers.front();
          int totalInputFrames = inputBuffer.size();
          int remainingInputFrames = totalInputFrames - audioDestination->inputIndex;

          currentFrames = std::min<int>(currentFrames, remainingInputFrames);
        } else {
          break;
        }
      }

      std::vector<float> outputBuffer(currentFrames * 2);
      std::vector<float> *inputBuffer;
      int inputIndex;
      std::vector<float> inputBufferCache;
      if (audioDestination->isRecording()) {
        inputBuffer = &audioDestination->inputBuffers.front();
        inputIndex = audioDestination->inputIndex;
      } else {
        inputBufferCache.resize(outputBuffer.size());
        inputBuffer = &inputBufferCache;
        inputIndex = 0;
      }
      audioDestination->render(currentFrames, outputBuffer.data(), inputBuffer->data() + inputIndex);

      std::vector<int16_t> outputS16Buffer(outputBuffer.size());
      uint8_t *dstData[] = {
        (uint8_t *)outputS16Buffer.data(),
      };
      const uint8_t *srcData[] = {
        (const uint8_t *)(&outputBuffer[0]),
        (const uint8_t *)(&outputBuffer[currentFrames]),
      };
      swr_convert(audioDestination->output_swr_ctx, dstData, currentFrames, srcData, currentFrames);
      memcpy((int16_t *)outputMlBuffer.ptr + audioDestination->outputIndex * 2, outputS16Buffer.data(), currentFrames * 2 * sizeof(int16_t));

      audioDestination->outputIndex += currentFrames;
      if (audioDestination->outputIndex >= totalOutputFrames) {
        MLResult result = MLAudioReleaseOutputStreamBuffer(audioDestination->outputHandle);
        if (result != MLResult_Ok) {
          std::cerr << "failed to release ml output buffer: " << result << std::endl;
        }

        audioDestination->outputMlBuffers.pop_front();
        audioDestination->outputIndex = 0;
      }

      if (audioDestination->isRecording()) {
        audioDestination->inputIndex += currentFrames;

        int totalInputFrames = inputBuffer->size();
        if (audioDestination->inputIndex >= totalInputFrames) {
          MLResult result = MLAudioReleaseInputStreamBuffer(audioDestination->inputHandle);
          if (result != MLResult_Ok) {
            std::cerr << "failed to release ml input buffer: " << result << std::endl;
          }

          audioDestination->inputMlBuffers.pop_front();
          audioDestination->inputBuffers.pop_front();
          audioDestination->inputIndex = 0;
        }
      }
    } else {
      break;
    }
  }
}

void outputBufferCallback(MLHandle handle, void *callback_context) {
  AudioDestinationMl *audioDestination = (AudioDestinationMl *)callback_context;

  {
    std::lock_guard<std::mutex> lock(audioDestination->mutex);

    MLAudioBuffer outputMlBuffer;
    MLResult result = MLAudioGetOutputStreamBuffer(
      audioDestination->outputHandle,
      &outputMlBuffer
    );
    if (result == MLResult_Ok) {
      audioDestination->outputMlBuffers.push_back(outputMlBuffer);

      processBuffers(audioDestination);
    } else {
      std::cerr << "failed to get ml output buffer: " << result << std::endl;
    }
  }
}

void inputBufferCallback(MLHandle handle, void *callback_context) {
  AudioDestinationMl *audioDestination = (AudioDestinationMl *)callback_context;

  {
    std::lock_guard<std::mutex> lock(audioDestination->mutex);

    MLAudioBuffer inputMlBuffer;
    MLResult result = MLAudioGetInputStreamBuffer(
      audioDestination->inputHandle,
      &inputMlBuffer
    );
    if (result == MLResult_Ok) {
      int numRawFrames = inputMlBuffer.size / sizeof(int16_t);
      int numConvertedFrames = mlBufferSize;
      std::vector<float> inputBuffer(numConvertedFrames);

      uint8_t *dstData[] = {
        (uint8_t *)inputBuffer.data(),
      };
      const uint8_t *srcData[] = {
        (const uint8_t *)inputMlBuffer.ptr,
      };
      swr_convert(audioDestination->input_swr_ctx, dstData, numConvertedFrames, srcData, numRawFrames);

      audioDestination->inputMlBuffers.push_back(inputMlBuffer);
      audioDestination->inputBuffers.push_back(std::move(inputBuffer));
      audioDestination->m_isRecording = true;

      processBuffers(audioDestination);
    } else {
      std::cerr << "failed to get ml input buffer: " << result << std::endl;
    }
  }
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
    {
      outputAudioBufferFormat.bits_per_sample = 16;
      outputAudioBufferFormat.channel_count = 2;
      outputAudioBufferFormat.sample_format = MLAudioSampleFormat_Int;
      outputAudioBufferFormat.samples_per_second = (uint32_t)sampleRate;
      outputAudioBufferFormat.valid_bits_per_sample = 16;

      uint32_t outputBufferSize = mlBufferSize*2*sizeof(int16_t);
      
      MLResult result = MLAudioCreateSoundWithOutputStream(
        &outputAudioBufferFormat,
        // nOutputBufferFramesPerChannel * 2 * sizeof(uint16_t),
        outputBufferSize,
        outputBufferCallback,
        this,
        &outputHandle
      );
      if (result != MLResult_Ok) {
        std::cerr << "failed to create ml output sound: " << result << std::endl;
      }
    }

    {
      inputAudioBufferFormat.bits_per_sample = 16;
      inputAudioBufferFormat.channel_count = 1;
      inputAudioBufferFormat.sample_format = MLAudioSampleFormat_Int;
      inputAudioBufferFormat.samples_per_second = 16000;
      inputAudioBufferFormat.valid_bits_per_sample = 16;

      uint32_t inputBufferSize = (uint32_t)av_rescale_rnd(mlBufferSize*sizeof(int16_t), inputAudioBufferFormat.samples_per_second, outputAudioBufferFormat.samples_per_second, AV_ROUND_UP);
      inputBufferSize += (sizeof(int16_t) - (inputBufferSize % sizeof(int16_t)));

      MLResult result = MLAudioCreateInputFromVoiceComm(
        &inputAudioBufferFormat,
        // nInputBufferFramesPerChannel * sizeof(uint16_t),
        inputBufferSize,
        inputBufferCallback,
        this,
        &inputHandle
      );
      if (result != MLResult_Ok) {
        std::cerr << "failed to create ml microphone input: " << result << std::endl;
      }
    }

    m_sampleRate = sampleRate;
    m_renderBus.setSampleRate(m_sampleRate);
    m_inputBus.setSampleRate(m_sampleRate);
    // configure();

    {
      int64_t src_ch_layout = AV_CH_LAYOUT_STEREO, dst_ch_layout = AV_CH_LAYOUT_STEREO;
      enum AVSampleFormat src_sample_fmt = AV_SAMPLE_FMT_FLTP, dst_sample_fmt = AV_SAMPLE_FMT_S16;
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
      enum AVSampleFormat src_sample_fmt = AV_SAMPLE_FMT_S16, dst_sample_fmt = AV_SAMPLE_FMT_FLTP;
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
      std::cerr << "failed to start ml output sound: " << result << std::endl;
    }
}

void AudioDestinationMl::stop()
{
    MLResult result = MLAudioStopSound(outputHandle);

    if (result == MLResult_Ok) {
      m_isPlaying = false;
    } else {
      std::cerr << "failed to stop ml output sound: " << result << std::endl;
    }

    if (isRecording()) {
      stopRecording();
    }
}

void AudioDestinationMl::startRecording()
{
    MLResult result = MLAudioStartInput(inputHandle);

    if (result == MLResult_Ok) {
      std::cout << "start recording" << std::endl;

      // m_isRecording = true; // only consider recording started on the first buffer emit
    } else {
      std::cerr << "failed to start ml input: " << result << std::endl;
    }
}

void AudioDestinationMl::stopRecording()
{
    MLResult result = MLAudioStopInput(inputHandle);

    if (result == MLResult_Ok) {
      m_isRecording = false;
    } else {
      std::cerr << "failed to stop ml input: " << result << std::endl;
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

    // Clamp values at 0d*b (i.e., [-1.0, 1.0])
    for (unsigned i = 0; i < m_renderBus.numberOfChannels(); ++i)
    {
        AudioChannel * channel = m_renderBus.channel(i);
        VectorMath::vclip(channel->data(), 1, &kLowThreshold, &kHighThreshold, channel->mutableData(), 1, numberOfFrames);
    }
}

} // namespace lab
