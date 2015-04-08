// Copyright (c) 2003-2013 Nick Porcino, All rights reserved.
// License is MIT: http://opensource.org/licenses/MIT

#include "LabSound/core/AudioNodeInput.h"
#include "LabSound/core/AudioNodeOutput.h"

#include "LabSound/extended/PWMNode.h"

#include "internal/AudioProcessor.h"
#include "internal/AudioBus.h"

#include <iostream>

using namespace WebCore;

namespace LabSound {

	////////////////////////////////////
    // Private PWMNode Implementation //
    ////////////////////////////////////

    class PWMNode::PWMNodeInternal : public WebCore::AudioProcessor
	{

    public:

        PWMNodeInternal(float sampleRate) : AudioProcessor(sampleRate), channels(1)
        {

        }

        virtual ~PWMNodeInternal() { }

        virtual void initialize() { }

        virtual void uninitialize() { }

        // Processes the source to destination bus.  The number of channels must match in source and destination.
        virtual void process(ContextRenderLock&, const WebCore::AudioBus* source, WebCore::AudioBus* destination, size_t framesToProcess) 
		{
            if (!channels)
                return;
            
            const float* carrierP = source->channel(0)->data();
            const float* modP = source->channel(1)->data();

            if (!modP && carrierP) 
			{
                destination->copyFrom(*source);
            }
            else 
			{
                float* destP = destination->channel(0)->mutableData();
                size_t n = framesToProcess;
                while (n--)
				{
                    float carrier = *carrierP++;
                    float mod = *modP++;
                    *destP++ = (carrier > mod) ? 1.0f : -1.0f;
                }
            }
        }

        virtual void reset() { }

        virtual void setNumberOfChannels(unsigned i)
		{
            channels = i;
        }

        virtual double tailTime() const override { return 0; }
        virtual double latencyTime() const override { return 0; }

        int channels;
    };

	////////////////////
    // Public PWMNode //
    ////////////////////

    PWMNode::PWMNode(float sampleRate) : WebCore::AudioBasicProcessorNode(sampleRate)
    {
        m_processor.reset(new PWMNodeInternal(sampleRate));

		internalNode = static_cast<PWMNodeInternal*>(m_processor.get()); // Currently unused 

        setNodeType((AudioNode::NodeType) LabSound::NodeTypePWM);

        addInput(std::unique_ptr<AudioNodeInput>(new WebCore::AudioNodeInput(this)));
        addOutput(std::unique_ptr<AudioNodeOutput>(new WebCore::AudioNodeOutput(this, 2))); 

        initialize();
    }

    PWMNode::~PWMNode()
    {
        uninitialize();
    }

    
} // LabSound

