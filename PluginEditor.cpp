// PluginEditor.cpp
#include "PluginEditor.h"

//==============================================================================
CloudLikeGranularEditor::CloudLikeGranularEditor (CloudLikeGranularProcessor& p)
    : AudioProcessorEditor (&p), processor (p)
{
    setSize (560, 280);

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
CloudLikeGranularEditor::~CloudLikeGranularEditor() = default;

void CloudLikeGranularEditor::handleAsyncUpdate()
{
    randomButton.setToggleState (false, juce::dontSendNotification);
}

void CloudLikeGranularEditor::setupKnob (Knob& k, const juce::String& name)
{
    addAndMakeVisible (k.slider);
    addAndMakeVisible (k.label);

    k.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    k.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 18);
    k.slider.setPopupDisplayEnabled (true, false, this);

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
                      10, 5, getWidth() - 20, 24,
                      juce::Justification::centred, 1);
}

void CloudLikeGranularEditor::resized()
{
    auto area = getLocalBounds().reduced (10);
    area.removeFromTop (30);

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

    auto buttonArea = buttonRow.reduced (10);
    auto half = buttonArea.getWidth() / 2;
    freezeButton.setBounds  (buttonArea.removeFromLeft (half).reduced (5));
    randomButton.setBounds  (buttonArea.reduced (5));
}