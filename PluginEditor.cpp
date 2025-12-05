// PluginEditor.cpp
#include "PluginEditor.h"
#include "BinaryData.h"

//==============================================================================
// KnobLookAndFeel implementation
KnobLookAndFeel::KnobLookAndFeel()
{
    // Load knob image from embedded binary data
    knobImage = juce::ImageCache::getFromMemory (BinaryData::knob_png,
                                                  BinaryData::knob_pngSize);
}

bool KnobLookAndFeel::loadKnobImage (const juce::File& imageFile)
{
    if (imageFile.existsAsFile())
    {
        knobImage = juce::ImageCache::getFromFile (imageFile);
        return knobImage.isValid();
    }
    return false;
}

void KnobLookAndFeel::drawRotarySlider (juce::Graphics& g, int x, int y, int width, int height,
                                       float sliderPosProportional, float rotaryStartAngle,
                                       float rotaryEndAngle, juce::Slider& slider)
{
    if (!knobImage.isValid())
    {
        // Fallback to default drawing if image not loaded
        LookAndFeel_V4::drawRotarySlider (g, x, y, width, height, sliderPosProportional,
                                         rotaryStartAngle, rotaryEndAngle, slider);
        return;
    }

    // Calculate which frame to display based on slider position
    int frameIndex = juce::jlimit (0, numFrames - 1,
                                   static_cast<int> (sliderPosProportional * numFrames));

    // Calculate source rectangle for the sprite frame
    juce::Rectangle<int> sourceRect (0, frameIndex * frameSize, frameSize, frameSize);

    // Calculate destination rectangle (centered in the given bounds)
    int size = juce::jmin (width, height);
    juce::Rectangle<int> destRect (x + (width - size) / 2,
                                   y + (height - size) / 2,
                                   size, size);

    // Draw the sprite frame
    g.drawImage (knobImage, destRect.toFloat(), sourceRect.toFloat());
}

//==============================================================================
CloudLikeGranularEditor::CloudLikeGranularEditor (CloudLikeGranularProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    setSize (560, 280);
    startTimerHz (30);  // Update button states at 30Hz

    setupKnob (positionKnob, "Position");
    setupKnob (sizeKnob,     "Size");
    setupKnob (pitchKnob,    "Pitch");
    setupKnob (densityKnob,  "Density");
    setupKnob (textureKnob,  "Texture");
    setupKnob (spreadKnob,   "Spread");
    setupKnob (feedbackKnob, "Feedback");
    setupKnob (reverbKnob,   "Reverb");
    setupKnob (mixKnob,      "Mix");

    addAndMakeVisible (freezeButton);
    addAndMakeVisible (randomButton);
    addAndMakeVisible (modeLabel);

    // Style mode label
    modeLabel.setFont (juce::Font (16.0f, juce::Font::bold));
    modeLabel.setColour (juce::Label::textColourId, juce::Colour::fromRGB (100, 255, 100));
    modeLabel.setJustificationType (juce::Justification::centred);

    // Style Freeze button with colors
    freezeButton.setColour (juce::ToggleButton::textColourId, juce::Colours::white);
    freezeButton.setColour (juce::ToggleButton::tickColourId, juce::Colour::fromRGB (100, 200, 255));
    freezeButton.setColour (juce::ToggleButton::tickDisabledColourId, juce::Colour::fromRGB (60, 60, 80));

    // Style Randomize button
    randomButton.setColour (juce::TextButton::buttonColourId, juce::Colour::fromRGB (60, 60, 80));
    randomButton.setColour (juce::TextButton::buttonOnColourId, juce::Colour::fromRGB (200, 100, 255));
    randomButton.setColour (juce::TextButton::textColourOffId, juce::Colours::white);
    randomButton.setColour (juce::TextButton::textColourOnId, juce::Colours::white);

    randomButton.onClick = [this]
    {
        if (auto* param = processor.apvts.getParameter ("randomize"))
        {
            randomButton.setToggleState (true, juce::dontSendNotification);
            param->beginChangeGesture();
            param->setValueNotifyingHost (1.0f);
            param->endChangeGesture();
            triggerAsyncUpdate();
        }
    };

    auto& apvts = processor.apvts;

    positionAttachment = std::make_unique<SliderAttachment> (apvts, "position", positionKnob.slider);
    sizeAttachment     = std::make_unique<SliderAttachment> (apvts, "size",     sizeKnob.slider);
    pitchAttachment    = std::make_unique<SliderAttachment> (apvts, "pitch",    pitchKnob.slider);
    densityAttachment  = std::make_unique<SliderAttachment> (apvts, "density",  densityKnob.slider);
    textureAttachment  = std::make_unique<SliderAttachment> (apvts, "texture",  textureKnob.slider);
    spreadAttachment   = std::make_unique<SliderAttachment> (apvts, "spread",   spreadKnob.slider);
    feedbackAttachment = std::make_unique<SliderAttachment> (apvts, "feedback", feedbackKnob.slider);
    reverbAttachment   = std::make_unique<SliderAttachment> (apvts, "reverb",   reverbKnob.slider);
    mixAttachment      = std::make_unique<SliderAttachment> (apvts, "mix",      mixKnob.slider);
    freezeAttachment   = std::make_unique<ButtonAttachment> (apvts, "freeze",   freezeButton);
}

// Default destructor (AsyncUpdater cleanup handled automatically)
CloudLikeGranularEditor::~CloudLikeGranularEditor()
{
    stopTimer();

    // Clear LookAndFeel from all sliders before destruction
    positionKnob.slider.setLookAndFeel (nullptr);
    sizeKnob.slider.setLookAndFeel (nullptr);
    pitchKnob.slider.setLookAndFeel (nullptr);
    densityKnob.slider.setLookAndFeel (nullptr);
    textureKnob.slider.setLookAndFeel (nullptr);
    spreadKnob.slider.setLookAndFeel (nullptr);
    feedbackKnob.slider.setLookAndFeel (nullptr);
    reverbKnob.slider.setLookAndFeel (nullptr);
    mixKnob.slider.setLookAndFeel (nullptr);
}

void CloudLikeGranularEditor::handleAsyncUpdate()
{
    randomButton.setToggleState (false, juce::dontSendNotification);
}

void CloudLikeGranularEditor::timerCallback()
{
    // Update freeze button visual state based on parameter
    bool freezeState = processor.apvts.getRawParameterValue ("freeze")->load() > 0.5f;
    if (freezeButton.getToggleState() != freezeState)
    {
        freezeButton.setToggleState (freezeState, juce::dontSendNotification);
        repaint();  // Repaint entire editor to update button highlights
    }

    // Update mode label based on mode parameter
    int mode = static_cast<int>(processor.apvts.getRawParameterValue ("mode")->load());
    juce::String modeText;

    switch (mode)
    {
        case 0: modeText = "MODE: Granular"; break;
        case 1: modeText = "MODE: WSOLA"; break;
        case 2: modeText = "MODE: Looping"; break;
        case 3: modeText = "MODE: Spectral"; break;
        default: modeText = "MODE: Unknown"; break;
    }

    if (modeLabel.getText() != modeText)
    {
        modeLabel.setText (modeText, juce::dontSendNotification);
    }
}

void CloudLikeGranularEditor::setupKnob (Knob& k, const juce::String& name)
{
    addAndMakeVisible (k.slider);
    addAndMakeVisible (k.label);

    k.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    k.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 18);
    k.slider.setPopupDisplayEnabled (true, false, this);

    // Apply custom LookAndFeel for sprite-based knob
    k.slider.setLookAndFeel (&knobLookAndFeel);

    k.label.setText (name, juce::dontSendNotification);
    k.label.attachToComponent (&k.slider, false);
    k.label.setJustificationType (juce::Justification::centred);
    k.label.setColour (juce::Label::textColourId, juce::Colours::white);
}

void CloudLikeGranularEditor::paint (juce::Graphics& g)
{
    auto bounds = getLocalBounds().toFloat();

    g.setGradientFill (juce::ColourGradient::vertical (
        juce::Colour::fromRGB (10, 10, 20),
        0.0f,
        juce::Colour::fromRGB (20, 20, 40),
        bounds.getHeight()
    ));
    g.fillAll();

    g.setColour (juce::Colours::white);
    g.setFont (juce::Font (20.0f, juce::Font::bold));
    g.drawFittedText ("Granular Texture",
                      10, 5, getWidth() - 160, 24,
                      juce::Justification::centred, 1);

    // Draw highlight around Freeze button when active
    if (freezeButton.getToggleState())
    {
        auto freezeBounds = freezeButton.getBounds().toFloat().expanded (2.0f);
        g.setColour (juce::Colour::fromRGB (100, 200, 255).withAlpha (0.6f));
        g.drawRoundedRectangle (freezeBounds, 4.0f, 2.5f);

        g.setColour (juce::Colour::fromRGB (100, 200, 255).withAlpha (0.15f));
        g.fillRoundedRectangle (freezeBounds, 4.0f);
    }

    // Draw subtle highlight around Randomize button when pressed
    if (randomButton.getToggleState())
    {
        auto randomBounds = randomButton.getBounds().toFloat().expanded (2.0f);
        g.setColour (juce::Colour::fromRGB (200, 100, 255).withAlpha (0.7f));
        g.drawRoundedRectangle (randomBounds, 4.0f, 2.5f);

        g.setColour (juce::Colour::fromRGB (200, 100, 255).withAlpha (0.2f));
        g.fillRoundedRectangle (randomBounds, 4.0f);
    }
}

void CloudLikeGranularEditor::resized()
{
    auto area = getLocalBounds().reduced (10);
    auto titleArea = area.removeFromTop (30);

    // Position mode label in top-right corner
    modeLabel.setBounds (titleArea.removeFromRight (140));

    auto leftArea  = area.removeFromLeft (150);
    auto rightArea = area;

    positionKnob.slider.setBounds (leftArea.reduced (10));

    auto buttonRow = rightArea.removeFromBottom (40);
    auto rowHeight = rightArea.getHeight() / 2;
    auto row1 = rightArea.removeFromTop (rowHeight);
    auto row2 = rightArea;

    auto colWidth = row1.getWidth() / 4;

    auto placeKnob = [] (Knob& k, juce::Rectangle<int> r)
    {
        k.slider.setBounds (r.reduced (10));
    };

    placeKnob (sizeKnob,     row1.removeFromLeft (colWidth));
    placeKnob (pitchKnob,    row1.removeFromLeft (colWidth));
    placeKnob (densityKnob,  row1.removeFromLeft (colWidth));
    placeKnob (textureKnob,  row1.removeFromLeft (colWidth));

    placeKnob (spreadKnob,   row2.removeFromLeft (colWidth));
    placeKnob (feedbackKnob, row2.removeFromLeft (colWidth));
    placeKnob (reverbKnob,   row2.removeFromLeft (colWidth));
    placeKnob (mixKnob,      row2.removeFromLeft (colWidth));

    auto buttonArea = buttonRow.reduced (5);
    auto half = buttonArea.getWidth() / 2;

    auto freezeArea = buttonArea.removeFromLeft (half).reduced (3);
    auto randomArea = buttonArea.reduced (3);

    freezeButton.setBounds (freezeArea);
    randomButton.setBounds (randomArea);

    // Make buttons more prominent with larger bounds
    freezeButton.setSize (freezeArea.getWidth(), juce::jmax (35, freezeArea.getHeight()));
    randomButton.setSize (randomArea.getWidth(), juce::jmax (35, randomArea.getHeight()));
}