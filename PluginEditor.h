// PluginEditor.h
#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
class CloudLikeGranularEditor  : public juce::AudioProcessorEditor
                               , public juce::AsyncUpdater
                               , public juce::Timer
{
public:
    CloudLikeGranularEditor (CloudLikeGranularProcessor&);
    ~CloudLikeGranularEditor() override;

    void paint (juce::Graphics&) override;
    void resized() override;
    void handleAsyncUpdate() override;
    void timerCallback() override;

private:
    CloudLikeGranularProcessor& processor;

    struct Knob
    {
        juce::Slider slider;
        juce::Label  label;
    };

    Knob positionKnob, sizeKnob, pitchKnob, densityKnob;
    Knob textureKnob, spreadKnob, feedbackKnob, reverbKnob, mixKnob;

    juce::ToggleButton freezeButton  { "Freeze" };
    juce::ToggleButton randomButton { "Randomize" };
    juce::Label modeLabel;  // Displays current mode (Granular/WSOLA)

    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment = juce::AudioProcessorValueTreeState::ButtonAttachment;

    std::unique_ptr<SliderAttachment> positionAttachment;
    std::unique_ptr<SliderAttachment> sizeAttachment;
    std::unique_ptr<SliderAttachment> pitchAttachment;
    std::unique_ptr<SliderAttachment> densityAttachment;
    std::unique_ptr<SliderAttachment> textureAttachment;
    std::unique_ptr<SliderAttachment> spreadAttachment;
    std::unique_ptr<SliderAttachment> feedbackAttachment;
    std::unique_ptr<SliderAttachment> reverbAttachment;
    std::unique_ptr<SliderAttachment> mixAttachment;
    std::unique_ptr<ButtonAttachment> freezeAttachment;

    void setupKnob (Knob& k, const juce::String& name);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularEditor)
};