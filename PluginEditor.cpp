// PluginEditor.cpp
#include "PluginEditor.h"

//==============================================================================
CloudLikeGranularEditor::CloudLikeGranularEditor (CloudLikeGranularProcessor& p)
    : AudioProcessorEditor (&p)
    , processor (p)
    , ePaperLookAndFeel (std::make_unique<EPaperLookAndFeel>())
{
    setSize (600, 400);  // E-Paper UI: 600x400
    startTimerHz (30);  // Update button states at 30Hz

    setupKnob (modeKnob, "Mode");
    modeKnob.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    modeKnob.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 22);
    modeKnob.slider.setNumDecimalPlacesToDisplay(0);  // Display integers only

    setupKnob (positionKnob, "Position");
    setupKnob (sizeKnob,     "Size");
    setupKnob (pitchKnob,    "Pitch");
    setupKnob (densityKnob,  "Density");
    setupKnob (textureKnob,  "Texture");
    setupKnob (spreadKnob,   "Spread");
    setupKnob (feedbackKnob, "Feedback");
    setupKnob (reverbKnob,   "Reverb");
    setupKnob (mixKnob,      "Mix");
    setupKnob (trigRateKnob, "Trig Rate");

    addAndMakeVisible (trigModeButton);
    addAndMakeVisible (freezeButton);
    addAndMakeVisible (randomButton);
    addAndMakeVisible (modeLabel);

    // Style mode label (E-Paper: ink blue for accent)
    modeLabel.setFont (juce::Font ("Courier New", 16.0f, juce::Font::bold));
    modeLabel.setColour (juce::Label::textColourId, juce::Colour::fromRGB (52, 73, 94));  // Ink blue
    modeLabel.setJustificationType (juce::Justification::centred);

    // Style TRIG Mode toggle button (E-Paper: matte black text)
    trigModeButton.setColour (juce::ToggleButton::textColourId, juce::Colour::fromRGB (26, 26, 26));
    trigModeButton.setColour (juce::ToggleButton::tickColourId, juce::Colour::fromRGB (26, 26, 26));
    trigModeButton.setColour (juce::ToggleButton::tickDisabledColourId, juce::Colour::fromRGB (160, 160, 160));
    trigModeButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Style Freeze button (E-Paper: matte black text)
    freezeButton.setColour (juce::ToggleButton::textColourId, juce::Colour::fromRGB (26, 26, 26));
    freezeButton.setColour (juce::ToggleButton::tickColourId, juce::Colour::fromRGB (26, 26, 26));
    freezeButton.setColour (juce::ToggleButton::tickDisabledColourId, juce::Colour::fromRGB (160, 160, 160));
    freezeButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Style Randomize button (E-Paper: matte black text, off-white background)
    randomButton.setColour (juce::TextButton::buttonColourId, juce::Colour::fromRGB (250, 250, 250));
    randomButton.setColour (juce::TextButton::buttonOnColourId, juce::Colour::fromRGB (224, 224, 224));
    randomButton.setColour (juce::TextButton::textColourOffId, juce::Colour::fromRGB (26, 26, 26));
    randomButton.setColour (juce::TextButton::textColourOnId, juce::Colour::fromRGB (26, 26, 26));
    randomButton.setColour (juce::ComboBox::outlineColourId, juce::Colour::fromRGB (26, 26, 26));
    randomButton.setLookAndFeel (ePaperLookAndFeel.get());

    // Trigger randomize parameter when button is clicked (MIDI/Ableton mappable)
    randomButton.onClick = [this]
    {
        if (auto* param = processor.apvts.getParameter ("randomize"))
        {
            param->beginChangeGesture();
            param->setValueNotifyingHost (1.0f);
            param->endChangeGesture();
            // Reset parameter back to 0 after a short delay
            param->setValueNotifyingHost (0.0f);
        }
    };

    auto& apvts = processor.apvts;

    modeAttachment     = std::make_unique<SliderAttachment> (apvts, "mode",     modeKnob.slider);
    positionAttachment = std::make_unique<SliderAttachment> (apvts, "position", positionKnob.slider);
    sizeAttachment     = std::make_unique<SliderAttachment> (apvts, "size",     sizeKnob.slider);
    pitchAttachment    = std::make_unique<SliderAttachment> (apvts, "pitch",    pitchKnob.slider);
    densityAttachment  = std::make_unique<SliderAttachment> (apvts, "density",  densityKnob.slider);
    textureAttachment  = std::make_unique<SliderAttachment> (apvts, "texture",  textureKnob.slider);
    spreadAttachment   = std::make_unique<SliderAttachment> (apvts, "spread",   spreadKnob.slider);
    feedbackAttachment = std::make_unique<SliderAttachment> (apvts, "feedback", feedbackKnob.slider);
    reverbAttachment   = std::make_unique<SliderAttachment> (apvts, "reverb",   reverbKnob.slider);
    mixAttachment      = std::make_unique<SliderAttachment> (apvts, "mix",      mixKnob.slider);
    trigRateAttachment = std::make_unique<SliderAttachment> (apvts, "trigRate", trigRateKnob.slider);
    trigModeAttachment = std::make_unique<ButtonAttachment> (apvts, "trigMode", trigModeButton);
    freezeAttachment   = std::make_unique<ButtonAttachment> (apvts, "freeze",   freezeButton);
}

CloudLikeGranularEditor::~CloudLikeGranularEditor()
{
    stopTimer();

    // Clear LookAndFeel references before unique_ptr destruction
    // This ensures proper cleanup order to prevent dangling pointers
    modeKnob.slider.setLookAndFeel (nullptr);
    positionKnob.slider.setLookAndFeel (nullptr);
    sizeKnob.slider.setLookAndFeel (nullptr);
    pitchKnob.slider.setLookAndFeel (nullptr);
    densityKnob.slider.setLookAndFeel (nullptr);
    textureKnob.slider.setLookAndFeel (nullptr);
    spreadKnob.slider.setLookAndFeel (nullptr);
    feedbackKnob.slider.setLookAndFeel (nullptr);
    reverbKnob.slider.setLookAndFeel (nullptr);
    mixKnob.slider.setLookAndFeel (nullptr);
    trigRateKnob.slider.setLookAndFeel (nullptr);
    trigModeButton.setLookAndFeel (nullptr);
    freezeButton.setLookAndFeel (nullptr);
    randomButton.setLookAndFeel (nullptr);

    // unique_ptr will automatically clean up ePaperLookAndFeel
}

void CloudLikeGranularEditor::timerCallback()
{
    // Update TRIG mode button text based on parameter
    bool trigMode = processor.apvts.getRawParameterValue ("trigMode")->load() > 0.5f;
    juce::String trigModeText = trigMode ? "Auto" : "Manual";
    if (trigModeButton.getButtonText() != trigModeText)
    {
        trigModeButton.setButtonText (trigModeText);
    }
    if (trigModeButton.getToggleState() != trigMode)
    {
        trigModeButton.setToggleState (trigMode, juce::dontSendNotification);
    }

    // Update freeze button visual state based on parameter
    bool freezeState = processor.apvts.getRawParameterValue ("freeze")->load() > 0.5f;
    if (freezeButton.getToggleState() != freezeState)
    {
        freezeButton.setToggleState (freezeState, juce::dontSendNotification);
        repaint();  // Repaint entire editor to update button highlights
    }

    // Update mode label and knob labels based on mode parameter
    int mode = static_cast<int>(processor.apvts.getRawParameterValue ("mode")->load());
    juce::String modeText;

    // Define knob labels for each mode
    juce::String posLabel, sizeLabel, pitchLabel, densityLabel, textureLabel;

    switch (mode)
    {
        case 0: // Granular
            modeText = "MODE: Granular";
            posLabel = "Position";
            sizeLabel = "Size";
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 1: // Pitch Shifter (Clouds-style)
            modeText = "MODE: Pitch Shifter";
            posLabel = "Position";
            sizeLabel = "Window";  // Small=Grainy, Large=Smooth
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 2: // Looping
            modeText = "MODE: Looping";
            posLabel = "LoopPos";
            sizeLabel = "LoopLen";
            pitchLabel = "Speed";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 3: // Spectral
            modeText = "MODE: Spectral";
            posLabel = "Delay";
            sizeLabel = "Window";
            pitchLabel = "FreqShft";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;

        case 4: // Oliverb
            modeText = "MODE: Oliverb";
            posLabel = "ModRate";
            sizeLabel = "Decay";
            pitchLabel = "Pitch";
            densityLabel = "ModDepth";
            textureLabel = "Diffuse";
            break;

        case 5: // Resonestor
            modeText = "MODE: Resonestor";
            posLabel = "Excite";
            sizeLabel = "Decay";
            pitchLabel = "Pitch";
            densityLabel = "Pattern";
            textureLabel = "Bright";
            break;

        case 6: // Beat Repeat
            modeText = "MODE: Beat Repeat";
            posLabel = "Capture";
            sizeLabel = "Length";
            pitchLabel = "Speed";
            densityLabel = "Rate";
            textureLabel = "Stutter";
            break;

        default:
            modeText = "MODE: Unknown";
            posLabel = "Position";
            sizeLabel = "Size";
            pitchLabel = "Pitch";
            densityLabel = "Density";
            textureLabel = "Texture";
            break;
    }

    // Update mode label
    if (modeLabel.getText() != modeText)
    {
        modeLabel.setText (modeText, juce::dontSendNotification);
    }

    // Update knob labels
    if (positionKnob.label.getText() != posLabel)
        positionKnob.label.setText (posLabel, juce::dontSendNotification);

    if (sizeKnob.label.getText() != sizeLabel)
        sizeKnob.label.setText (sizeLabel, juce::dontSendNotification);

    if (pitchKnob.label.getText() != pitchLabel)
        pitchKnob.label.setText (pitchLabel, juce::dontSendNotification);

    if (densityKnob.label.getText() != densityLabel)
        densityKnob.label.setText (densityLabel, juce::dontSendNotification);

    if (textureKnob.label.getText() != textureLabel)
        textureKnob.label.setText (textureLabel, juce::dontSendNotification);
}

void CloudLikeGranularEditor::setupKnob (Knob& k, const juce::String& name)
{
    addAndMakeVisible (k.slider);
    addAndMakeVisible (k.label);

    k.slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
    k.slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 60, 22);
    k.slider.setPopupDisplayEnabled (true, false, this);

    // Apply E-Paper LookAndFeel
    k.slider.setLookAndFeel (ePaperLookAndFeel.get());

    // E-Paper styling for text box and label
    k.slider.setColour (juce::Slider::textBoxTextColourId, juce::Colour::fromRGB (102, 102, 102));
    k.slider.setColour (juce::Slider::textBoxBackgroundColourId, juce::Colours::transparentBlack);
    k.slider.setColour (juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);

    k.label.setText (name, juce::dontSendNotification);
    k.label.attachToComponent (&k.slider, false);
    k.label.setJustificationType (juce::Justification::centred);
    k.label.setColour (juce::Label::textColourId, juce::Colour::fromRGB (26, 26, 26));
    k.label.setFont (juce::Font ("Courier New", 12.0f, juce::Font::plain));
}

void CloudLikeGranularEditor::paint (juce::Graphics& g)
{
    // E-Paper UI: Off-white background
    g.fillAll (juce::Colour::fromRGB (250, 250, 250));

    // Draw border
    g.setColour (juce::Colour::fromRGB (26, 26, 26));
    g.drawRect (getLocalBounds(), 2);

    // Draw separator line below mode label
    g.setColour (juce::Colour::fromRGB (224, 224, 224));
    g.fillRect (10, 45, getWidth() - 20, 1);

    // Draw separator line above buttons
    g.setColour (juce::Colour::fromRGB (224, 224, 224));
    g.fillRect (10, getHeight() - 60, getWidth() - 20, 1);

    // Draw subtle border around Freeze button when active (E-Paper style)
    if (freezeButton.getToggleState())
    {
        auto freezeBounds = freezeButton.getBounds().toFloat().reduced (1.0f);
        g.setColour (juce::Colour::fromRGB (26, 26, 26));
        g.drawRoundedRectangle (freezeBounds, 0.0f, 2.0f);
    }
}

void CloudLikeGranularEditor::resized()
{
    auto area = getLocalBounds().reduced (10);

    // E-Paper UI: Mode label at top center
    auto titleArea = area.removeFromTop (35);
    modeLabel.setBounds (titleArea.withTrimmedLeft (titleArea.getWidth() / 4)
                                   .withTrimmedRight (titleArea.getWidth() / 4));

    // E-Paper UI: 3 rows × 5 columns grid (added 3rd row for TRIG rate)
    auto buttonRow = area.removeFromBottom (50);
    auto knobArea = area.reduced (0, 10);

    auto rowHeight = knobArea.getHeight() / 3;
    auto row1 = knobArea.removeFromTop (rowHeight);
    auto row2 = knobArea.removeFromTop (rowHeight);
    auto row3 = knobArea;

    auto colWidth = row1.getWidth() / 5;

    auto placeKnob = [] (Knob& k, juce::Rectangle<int> r)
    {
        // Reduce knob size to half by increasing margin
        k.slider.setBounds (r.reduced (25));
    };

    // Row 1: Position, Density, Size, Texture, Pitch
    placeKnob (positionKnob, row1.removeFromLeft (colWidth));
    placeKnob (densityKnob,  row1.removeFromLeft (colWidth));
    placeKnob (sizeKnob,     row1.removeFromLeft (colWidth));
    placeKnob (textureKnob,  row1.removeFromLeft (colWidth));
    placeKnob (pitchKnob,    row1.removeFromLeft (colWidth));

    // Row 2: Spread, Feedback, Reverb, Mix, Mode
    placeKnob (spreadKnob,   row2.removeFromLeft (colWidth));
    placeKnob (feedbackKnob, row2.removeFromLeft (colWidth));
    placeKnob (reverbKnob,   row2.removeFromLeft (colWidth));
    placeKnob (mixKnob,      row2.removeFromLeft (colWidth));
    placeKnob (modeKnob,     row2.removeFromLeft (colWidth));

    // Row 3: TRIG Rate (centered, single knob)
    auto trigRateArea = row3.withSizeKeepingCentre (colWidth, rowHeight);
    placeKnob (trigRateKnob, trigRateArea);

    // E-Paper UI: Buttons at bottom (3 buttons: TrigMode, Freeze, Randomize)
    auto buttonArea = buttonRow.reduced (5);
    auto third = buttonArea.getWidth() / 3;

    auto trigModeArea = buttonArea.removeFromLeft (third).reduced (3);
    auto freezeArea = buttonArea.removeFromLeft (third).reduced (3);
    auto randomArea = buttonArea.reduced (3);

    trigModeButton.setBounds (trigModeArea);
    freezeButton.setBounds (freezeArea);
    randomButton.setBounds (randomArea);
}