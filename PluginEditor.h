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

    // Custom LookAndFeel for image-based knobs
    class CustomKnobLookAndFeel : public juce::LookAndFeel_V4
    {
    public:
        CustomKnobLookAndFeel()
        {
            // Try to load knob image from project directory
            juce::File knobFile = juce::File::getCurrentWorkingDirectory().getChildFile("knob.png");
            if (!knobFile.existsAsFile())
            {
                // Try parent directories
                knobFile = juce::File::getCurrentWorkingDirectory().getParentDirectory().getChildFile("knob.png");
            }

            if (knobFile.existsAsFile())
            {
                knobImage = juce::ImageCache::getFromFile(knobFile);
            }
        }

        void drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                              float sliderPos, float rotaryStartAngle, float rotaryEndAngle,
                              juce::Slider& slider) override
        {
            if (knobImage.isValid())
            {
                // Image-based knob rendering
                const int frameCount = knobImage.getHeight() / knobImage.getWidth();
                const int frameHeight = knobImage.getWidth();
                const int frameIndex = static_cast<int>(sliderPos * (frameCount - 1));

                const int knobSize = juce::jmin(width, height);
                const int centerX = x + (width - knobSize) / 2;
                const int centerY = y + (height - knobSize) / 2;

                g.drawImage(knobImage,
                           centerX, centerY, knobSize, knobSize,
                           0, frameIndex * frameHeight, knobImage.getWidth(), frameHeight);
            }
            else
            {
                // Fallback: custom drawn knob
                auto bounds = juce::Rectangle<int> (x, y, width, height).toFloat().reduced (10);
                auto radius = juce::jmin (bounds.getWidth(), bounds.getHeight()) / 2.0f;
                auto toAngle = rotaryStartAngle + sliderPos * (rotaryEndAngle - rotaryStartAngle);
                auto lineW = juce::jmin (8.0f, radius * 0.5f);
                auto arcRadius = radius - lineW * 0.5f;

                // Draw outer circle
                g.setColour (juce::Colour::fromRGB (60, 60, 80));
                g.fillEllipse (bounds);

                // Draw inner circle (knob body)
                auto innerBounds = bounds.reduced (lineW * 0.5f);
                g.setColour (juce::Colour::fromRGB (40, 40, 60));
                g.fillEllipse (innerBounds);

                // Draw arc track
                juce::Path backgroundArc;
                backgroundArc.addCentredArc (bounds.getCentreX(),
                                            bounds.getCentreY(),
                                            arcRadius,
                                            arcRadius,
                                            0.0f,
                                            rotaryStartAngle,
                                            rotaryEndAngle,
                                            true);

                g.setColour (juce::Colour::fromRGB (80, 80, 100));
                g.strokePath (backgroundArc, juce::PathStrokeType (lineW, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));

                // Draw value arc
                if (toAngle > rotaryStartAngle)
                {
                    juce::Path valueArc;
                    valueArc.addCentredArc (bounds.getCentreX(),
                                           bounds.getCentreY(),
                                           arcRadius,
                                           arcRadius,
                                           0.0f,
                                           rotaryStartAngle,
                                           toAngle,
                                           true);

                    g.setColour (juce::Colour::fromRGB (100, 200, 255));
                    g.strokePath (valueArc, juce::PathStrokeType (lineW, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
                }

                // Draw indicator dot
                juce::Point<float> dotPoint (bounds.getCentreX() + arcRadius * std::cos (toAngle - juce::MathConstants<float>::halfPi),
                                            bounds.getCentreY() + arcRadius * std::sin (toAngle - juce::MathConstants<float>::halfPi));

                g.setColour (juce::Colours::white);
                g.fillEllipse (juce::Rectangle<float> (lineW, lineW).withCentre (dotPoint));
            }
        }

    private:
        juce::Image knobImage;
    };

    CustomKnobLookAndFeel customKnobLookAndFeel;

    struct Knob
    {
        juce::Slider slider;
        juce::Label  label;
    };

    Knob positionKnob, sizeKnob, pitchKnob, densityKnob;
    Knob textureKnob, spreadKnob, feedbackKnob, reverbKnob, mixKnob;

    juce::ToggleButton freezeButton  { "Freeze" };
    juce::ToggleButton randomButton { "Randomize" };
    juce::Label modeLabel;  // Displays current mode (Granular/WSOLA/Looping/Spectral)
    juce::ComboBox modeSelector;  // Dropdown for mode selection

    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment = juce::AudioProcessorValueTreeState::ButtonAttachment;
    using ComboBoxAttachment = juce::AudioProcessorValueTreeState::ComboBoxAttachment;

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
    std::unique_ptr<ComboBoxAttachment> modeAttachment;

    void setupKnob (Knob& k, const juce::String& name);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (CloudLikeGranularEditor)
};