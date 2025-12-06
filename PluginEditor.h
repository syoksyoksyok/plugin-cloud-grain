// PluginEditor.h
#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
// E-Paper LookAndFeel for rotary knobs with tick marks
class EPaperLookAndFeel : public juce::LookAndFeel_V4
{
public:
    EPaperLookAndFeel()
    {
        setColour (juce::Slider::thumbColourId, juce::Colour::fromRGB (26, 26, 26));
        setColour (juce::Slider::rotarySliderFillColourId, juce::Colour::fromRGB (26, 26, 26));
        setColour (juce::Slider::rotarySliderOutlineColourId, juce::Colour::fromRGB (224, 224, 224));
    }

    void drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                          float sliderPos, float rotaryStartAngle, float rotaryEndAngle,
                          juce::Slider& slider) override
    {
        auto radius = juce::jmin (width / 2, height / 2) - 4.0f;
        auto centreX = x + width * 0.5f;
        auto centreY = y + height * 0.5f;
        auto rx = centreX - radius;
        auto ry = centreY - radius;
        auto rw = radius * 2.0f;
        auto angle = rotaryStartAngle + sliderPos * (rotaryEndAngle - rotaryStartAngle);

        // Draw outer circle (outline)
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.drawEllipse (rx, ry, rw, rw, 2.0f);

        // Draw tick marks
        g.setColour (juce::Colour::fromRGB (224, 224, 224));
        for (int i = 0; i < 5; ++i)
        {
            auto tickAngle = rotaryStartAngle + i * (rotaryEndAngle - rotaryStartAngle) / 4.0f;
            auto tickRadius = radius - 2.0f;
            auto tickLength = 6.0f;

            auto tickX1 = centreX + tickRadius * std::cos (tickAngle - juce::MathConstants<float>::halfPi);
            auto tickY1 = centreY + tickRadius * std::sin (tickAngle - juce::MathConstants<float>::halfPi);
            auto tickX2 = centreX + (tickRadius - tickLength) * std::cos (tickAngle - juce::MathConstants<float>::halfPi);
            auto tickY2 = centreY + (tickRadius - tickLength) * std::sin (tickAngle - juce::MathConstants<float>::halfPi);

            g.drawLine (tickX1, tickY1, tickX2, tickY2, 2.0f);
        }

        // Draw center dot
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.fillEllipse (centreX - 3.0f, centreY - 3.0f, 6.0f, 6.0f);

        // Draw indicator line
        auto pointerLength = radius - 8.0f;
        auto pointerThickness = 3.0f;
        juce::Path p;
        auto pointerX = centreX + pointerLength * std::cos (angle - juce::MathConstants<float>::halfPi);
        auto pointerY = centreY + pointerLength * std::sin (angle - juce::MathConstants<float>::halfPi);

        p.startNewSubPath (centreX, centreY);
        p.lineTo (pointerX, pointerY);
        g.strokePath (p, juce::PathStrokeType (pointerThickness, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));
    }

    void drawButtonBackground (juce::Graphics& g, juce::Button& button, const juce::Colour& backgroundColour,
                              bool shouldDrawButtonAsHighlighted, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();

        // E-Paper button background
        g.setColour (juce::Colour::fromRGB (250, 250, 250));
        g.fillRect (bounds);

        // E-Paper button border
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.drawRect (bounds, 2.0f);
    }

    void drawToggleButton (juce::Graphics& g, juce::ToggleButton& button,
                          bool shouldDrawButtonAsHighlighted, bool shouldDrawButtonAsDown) override
    {
        auto bounds = button.getLocalBounds().toFloat();
        auto fontSize = juce::jmin (15.0f, button.getHeight() * 0.4f);
        auto tickWidth = fontSize * 1.2f;

        // Draw checkbox
        auto checkboxBounds = bounds.removeFromLeft (tickWidth).reduced (2.0f);
        g.setColour (juce::Colour::fromRGB (250, 250, 250));
        g.fillRect (checkboxBounds);

        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.drawRect (checkboxBounds, 2.0f);

        if (button.getToggleState())
        {
            auto tick = checkboxBounds.reduced (4.0f);
            g.setColour (juce::Colour::fromRGB (26, 26, 26));
            g.fillRect (tick);
        }

        // Draw text
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.setFont (juce::Font ("Courier New", fontSize, juce::Font::bold));
        g.drawText (button.getButtonText(), bounds.withTrimmedLeft (2.0f),
                   juce::Justification::centredLeft, true);
    }
};

//==============================================================================
// Grain Visualizer Component
class GrainVisualizer : public juce::Component, public juce::Timer
{
public:
    GrainVisualizer(CloudLikeGranularProcessor& proc) : processor(proc)
    {
        startTimerHz(30);  // 30 FPS
    }

    void paint(juce::Graphics& g) override
    {
        // E-Paper background
        g.fillAll(juce::Colour::fromRGB(250, 250, 250));

        // Draw grains
        for (const auto& grain : grainData)
        {
            if (!grain.active) continue;

            float env = grain.envelope;
            float x = grain.position * getWidth();
            float y = getHeight() / 2.0f + (grain.pitch / 24.0f) * (getHeight() / 3.0f);

            // Lighter grayscale particles
            float alpha = env * 0.6f + 0.15f;
            int grayValue = static_cast<int>(120 + env * 40);
            auto color = juce::Colour::fromRGBA(grayValue, grayValue, grayValue, static_cast<juce::uint8>(alpha * 255));

            // Draw particle
            float radius = 15.0f * env;
            g.setColour(color);
            g.fillEllipse(x - radius, y - radius, radius * 2, radius * 2);

            // Subtle outline
            g.setColour(juce::Colour::fromRGBA(80, 80, 80, static_cast<juce::uint8>(alpha * 102)));
            g.drawEllipse(x - radius, y - radius, radius * 2, radius * 2, 1.0f);
        }
    }

    void timerCallback() override
    {
        processor.copyGrainDataForVisualization(grainData, activeCount);
        repaint();
    }

    int getActiveGrainCount() const { return activeCount; }

private:
    CloudLikeGranularProcessor& processor;
    std::array<CloudLikeGranularProcessor::GrainVisualizationData, CloudLikeGranularProcessor::maxVisualGrains> grainData;
    int activeCount = 0;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(GrainVisualizer)
};

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

    EPaperLookAndFeel ePaperLookAndFeel;  // Custom LookAndFeel for e-paper aesthetic

    struct Knob
    {
        juce::Slider slider;
        juce::Label  label;
    };

    Knob modeKnob, positionKnob, sizeKnob, pitchKnob, densityKnob;
    Knob textureKnob, spreadKnob, feedbackKnob, reverbKnob, mixKnob;

    juce::ToggleButton freezeButton  { "Freeze" };
    juce::ToggleButton randomButton { "Randomize" };
    juce::Label modeLabel;  // Displays current mode (Granular/Pitch Shifter/etc.)

    // Grain Visualizer
    GrainVisualizer grainVisualizer;
    juce::Label grainCountLabel;
    juce::Label densityInfoLabel;

    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment = juce::AudioProcessorValueTreeState::ButtonAttachment;

    std::unique_ptr<SliderAttachment> modeAttachment;
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