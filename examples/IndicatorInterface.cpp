/**
 * @file IndicatorInterface.cpp
 * @brief Implementation of user interface with OLED display and input handling
 * @author Claude Code Session 20250720_221011
 * @date 2025-01-20
 * @details Implements IndicatorInterface class for managing OLED display output,
 *          button/encoder input through PCF8575 I/O expander, and LED indicators.
 * 
 * @section dependencies Dependencies
 * - IndicatorInterface.h for class definition
 * - U8g2lib for OLED display control
 * - PCF8575 library for I/O expansion
 * - Wire library for I2C communication
 * 
 * @section hardware Hardware Requirements
 * - SSD1306 OLED display (128x64)
 * - PCF8575 I/O expander for buttons/encoders
 * - Optional interrupt pin for input detection
 * - LEDs connected to PCF8575 outputs
 */

#include "IndicatorInterface.h"


// Static instance for interrupt handling
IndicatorInterface* IndicatorInterface::_instance = nullptr;

IndicatorInterface::IndicatorInterface(TwoWire& i2cBus, uint8_t pcf_i2cAddress, int intPin)
    : _i2cBus(&i2cBus), _pcf_i2cAddress(pcf_i2cAddress), _intPin(intPin), _pcf8575(pcf_i2cAddress),
      _directionMask(0x0000), _modeMask(0x0000), _currentState(0xFFFF), _lastState(0xFFFF),
      _lastReadTime(0), _pollInterval(50), _interruptFlag(false), _useInterrupts(false),
      _interruptCallback(nullptr),
      _oledSleepDelay(30000), _oledLines(3), _textBufferSize(0), _oledOn(true),
      _oledBlink(false), _blinkTimeOn(500), _blinkTimeOff(500), _lastBlinkTime(0),
      _blinkState(true), _lastActivityTime(0), _oledSleeping(false),
      _lastScrollTime(0), _scrollDelay(SCROLL_UPDATE_DELAY_MS), _charWidth(6), _lineHeight(12),
      _maxCharsPerLine(21),
      _savedTextBufferSize(0), _savedOledLines(3), _isBlinkingOK(false), 
      _isBlinkingCross(false), _blinkDelayTime(500), _lastBlinkToggle(0), _blinkShowSpecial(true)  {
    
    // Set static instance for interrupt handling
    _instance = this;
    
    // Configure interrupt usage
    _useInterrupts = (intPin >= 0);

    // Initialize scroll offsets
    for (int i = 0; i < 5; i++) {
        _scrollOffset[i] = 0;
    }
    
}

U8G2_SH1106_128X64_NONAME_F_HW_I2C IndicatorInterface::u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

IndicatorInterface::~IndicatorInterface() {
    if (_useInterrupts && _intPin >= 0) {
        detachInterrupt(digitalPinToInterrupt(_intPin));
    }
    _instance = nullptr;
}

bool IndicatorInterface::begin() {
    // Initialize I2C if not already done
    if (!_i2cBus) {
        return false;
    }
    
    // Initialize PCF8575
    if (!_pcf8575.begin()) {
        return false;
    }
    
    // Configure interrupt pin if specified
    if (_useInterrupts) {
        _configureInterruptPin();
    }
    
    // Initialize all pins as inputs (HIGH state)
    _pcf8575.write16(0xFFFF);
    delay(100);
    _clearInterrupt();
    
    // Read initial state
    _currentState = _readPCF();
    _lastState = _currentState;
    _lastReadTime = millis();

    //OLED INIT
    u8g2.begin();
    _initOLED();
    
    return true;
}

void IndicatorInterface::_configureInterruptPin() {
    if (_intPin < 0) return;
    
    // Configure pin based on ESP32 capabilities
    if (_intPin == 34 || _intPin == 35 || _intPin == 36 || _intPin == 39) {
        // Input-only pins without internal pull-up
        pinMode(_intPin, INPUT);
        // Note: External pull-up resistor (4.7kΩ) required!
    } else {
        // Regular GPIO pins with internal pull-up
        pinMode(_intPin, INPUT_PULLUP);
    }
    
    // Attach interrupt
    attachInterrupt(digitalPinToInterrupt(_intPin), _staticInterruptHandler, FALLING);
}

void IndicatorInterface::setDirection(uint16_t directionMask) {
    _directionMask = directionMask;
    
    // Update PCF8575 state to reflect direction changes
    uint16_t newState = _currentState;
    
    // Set input pins HIGH (input mode for PCF8575)
    for (int i = 0; i < 16; i++) {
        if (!isOutput(i)) {
            newState |= (1 << i);
        }
    }
    
    _writePCF(newState);
}

void IndicatorInterface::setMode(uint16_t modeMask) {
    _modeMask = modeMask;
}

void IndicatorInterface::setPortNames(const std::map<std::string, uint8_t>& portNames) {
    _portNames = portNames;
    
    // Build reverse mapping
    _portNumbers.clear();
    for (const auto& pair : _portNames) {
        _portNumbers[pair.second] = pair.first;
    }
}

void IndicatorInterface::setPortName(const std::string& name, uint8_t portNumber) {
    if (portNumber > 15) return;
    
    _portNames[name] = portNumber;
    _portNumbers[portNumber] = name;
}

bool IndicatorInterface::writePort(const std::string& portName, bool state) {
    auto it = _portNames.find(portName);
    if (it == _portNames.end()) {
        return false;
    }
    return writePort(it->second, state);
}

bool IndicatorInterface::writePort(uint8_t portNumber, bool state) {
    if (portNumber > 15 || !isOutput(portNumber)) {
        return false;
    }
    
    // Apply mode logic (inversion if needed)
    bool actualState = _applyModeLogic(portNumber, state);
    
    // Update current state
    uint16_t newState = _currentState;
    
    // Always keep input pins HIGH
    for (int i = 0; i < 16; i++) {
        if (!isOutput(i)) {
            newState |= (1 << i);
        }
    }
    
    // Set the specific output pin
    if (actualState) {
        newState |= (1 << portNumber);
    } else {
        newState &= ~(1 << portNumber);
    }
    
    _writePCF(newState);
    return true;
}

void IndicatorInterface::writePorts(uint16_t portMask) {
    uint16_t newState = _currentState;
    
    // Apply direction mask - only write to outputs
    for (int i = 0; i < 16; i++) {
        if (isOutput(i)) {
            bool state = (portMask >> i) & 0x01;
            bool actualState = _applyModeLogic(i, state);
            
            if (actualState) {
                newState |= (1 << i);
            } else {
                newState &= ~(1 << i);
            }
        } else {
            // Keep input pins HIGH
            newState |= (1 << i);
        }
    }
    
    _writePCF(newState);
}

void IndicatorInterface::setAllOutputs(bool state) {
    uint16_t newState = _currentState;
    
    for (int i = 0; i < 16; i++) {
        if (isOutput(i)) {
            bool actualState = _applyModeLogic(i, state);
            
            if (actualState) {
                newState |= (1 << i);
            } else {
                newState &= ~(1 << i);
            }
        } else {
            // Keep input pins HIGH
            newState |= (1 << i);
        }
    }
    
    _writePCF(newState);
}

void IndicatorInterface::setAllOutputsHigh() {
    setAllOutputs(true);
}

void IndicatorInterface::setAllOutputsLow() {
    setAllOutputs(false);
}

uint16_t IndicatorInterface::getCurrentState() {
    if (_useInterrupts) {
        // In interrupt mode, state is updated automatically
        return _currentState;
    } else {
        // In polling mode, read current state
        if (millis() - _lastReadTime >= _pollInterval) {
            _updateState();
        }
        return _currentState;
    }
}

bool IndicatorInterface::readPort(const std::string& portName) {
    auto it = _portNames.find(portName);
    if (it == _portNames.end()) {
        return false;
    }
    return readPort(it->second);
}

bool IndicatorInterface::readPort(uint8_t portNumber) {
    if (portNumber > 15) {
        return false;
    }
    
    uint16_t currentState = getCurrentState();
    bool rawState = (currentState >> portNumber) & 0x01;
    
    // Apply mode logic (reverse inversion for reading)
    return _reverseModeLogic(portNumber, rawState);
}

bool IndicatorInterface::isOutput(uint8_t portNumber) {
    return (_directionMask >> portNumber) & 0x01;
}

bool IndicatorInterface::isInput(uint8_t portNumber) {
    return !isOutput(portNumber);
}

bool IndicatorInterface::isInverted(uint8_t portNumber) {
    return (_modeMask >> portNumber) & 0x01;
}

uint8_t IndicatorInterface::getPortNumber(const std::string& portName) {
    auto it = _portNames.find(portName);
    return (it != _portNames.end()) ? it->second : 255;
}

std::string IndicatorInterface::getPortName(uint8_t portNumber) {
    auto it = _portNumbers.find(portNumber);
    return (it != _portNumbers.end()) ? it->second : "";
}

// void IndicatorInterface::handleInterrupt() {
//     if (_interruptFlag) {
//         _interruptFlag = false;
//         _updateState();
//     }
// }

void IndicatorInterface::setInterruptCallback(void (*callback)(uint16_t currentState, uint16_t changedPins)) {
    _interruptCallback = callback;
}

void IndicatorInterface::printPortStates() {
    uint16_t state = getCurrentState();
    
    Serial.println("=== Port States ===");
    Serial.print("Raw state: 0x");
    Serial.println(state, HEX);
    
    for (int i = 15; i >= 0; i--) {
        bool rawState = (state >> i) & 0x01;
        bool logicalState = _reverseModeLogic(i, rawState);
        
        Serial.print("P");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(rawState ? "HIGH" : "LOW");
        Serial.print(" (");
        Serial.print(isOutput(i) ? "OUT" : "IN");
        if (isInverted(i)) Serial.print(",INV");
        Serial.print(") = ");
        Serial.print(logicalState ? "TRUE" : "FALSE");
        
        std::string name = getPortName(i);
        if (!name.empty()) {
            Serial.print(" [");
            Serial.print(name.c_str());
            Serial.print("]");
        }
        Serial.println();
    }
}

void IndicatorInterface::printConfiguration() {
    Serial.println("=== Configuration ===");
    Serial.print("I2C Address: 0x");
    Serial.println(_pcf_i2cAddress, HEX);
    Serial.print("INT Pin: ");
    Serial.println(_intPin);
    Serial.print("Use Interrupts: ");
    Serial.println(_useInterrupts ? "YES" : "NO");
    Serial.print("Direction Mask: 0x");
    Serial.println(_directionMask, HEX);
    Serial.print("Mode Mask: 0x");
    Serial.println(_modeMask, HEX);
    
    Serial.println("Port Names:");
    for (const auto& pair : _portNames) {
        Serial.print("  ");
        Serial.print(pair.first.c_str());
        Serial.print(" = P");
        Serial.println(pair.second);
    }
}

// Private methods
void IndicatorInterface::_updateState() {
    uint16_t newState = _readPCF();
    uint16_t changedPins = _currentState ^ newState;
    
    _lastState = _currentState;
    _currentState = newState;
    _lastReadTime = millis();
    
    // Call interrupt callback if pins changed
    if (changedPins != 0 && _interruptCallback) {
        _interruptCallback(_currentState, changedPins);
    }
}

void IndicatorInterface::_clearInterrupt() {
    _pcf8575.read16();
    delay(1);
    _pcf8575.read16();
}

uint16_t IndicatorInterface::_readPCF() {
    return _pcf8575.read16();
}

void IndicatorInterface::_writePCF(uint16_t state) {
    _pcf8575.write16(state);
    delay(5);
    _clearInterrupt();
    _currentState = state;
}

bool IndicatorInterface::_applyModeLogic(uint8_t portNumber, bool state) {
    // Fixed: Normal mode = no inversion, Inverted mode = invert
    return isInverted(portNumber) ? !state : state;
}

bool IndicatorInterface::_reverseModeLogic(uint8_t portNumber, bool state) {
    // Fixed: For reading, apply same logic as writing
    return isInverted(portNumber) ? !state : state;
}

void IRAM_ATTR IndicatorInterface::_staticInterruptHandler() {
    if (_instance) {
        _instance->_interruptFlag = true;
    }
}


void IndicatorInterface::setPortInverted(const std::string& portName, bool inverted) {
    auto it = _portNames.find(portName);
    if (it != _portNames.end()) {
        setPortInverted(it->second, inverted);
    }
}

void IndicatorInterface::setPortInverted(uint8_t portNumber, bool inverted) {
    if (portNumber > 15) return;
    
    if (inverted) {
        _modeMask |= (1 << portNumber);   // Set bit to 1 for inverted
    } else {
        _modeMask &= ~(1 << portNumber);  // Set bit to 0 for normal
    }
}


void IndicatorInterface::_initOLED() {
    u8g2.enableUTF8Print();
    u8g2.clearBuffer();
    _calculateDisplayParams();
    u8g2.sendBuffer();
    _lastActivityTime = millis();
}

void IndicatorInterface::setOledSleepDelay(long sleepDelay) {
    _oledSleepDelay = sleepDelay;
    _wakeOLED();
}

void IndicatorInterface::setOledMode(int lines) {
    if (lines < 1) lines = 1;
    if (lines > 5) lines = 5;
    
    _oledLines = lines;
    _calculateDisplayParams();
    _wakeOLED();
}

void IndicatorInterface::setOledModeSmall(int lines, bool useSmallFont) {
    if (lines < 1) lines = 1;
    if (lines > 5) lines = 5;
    
    _oledLines = lines;
    
    if (useSmallFont) {
        // Force smallest font regardless of line count
        u8g2.setFont(u8g2_font_4x6_t_cyrillic);
        _lineHeight = 10;
        _charWidth = 4;
        _maxCharsPerLine = 32;
    } else {
        _calculateDisplayParams();
    }
    _wakeOLED();
}

void IndicatorInterface::printText(String buffer[], int bufferSize) {
    _textBufferSize = min(bufferSize, 5);
    
    for (int i = 0; i < _textBufferSize; i++) {
        // Only reset scroll offset if the text actually changed
        if (_textBuffer[i] != buffer[i]) {
            _textBuffer[i] = buffer[i];
            _scrollOffset[i] = 0; // Reset only when text changes
        }
    }
    
    // Clear unused lines
    for (int i = _textBufferSize; i < 5; i++) {
        _textBuffer[i] = "";
        _scrollOffset[i] = 0;
    }
    
    _lastActivityTime = millis();
    _wakeOLED();
    _updateOLEDDisplay();
}


void IndicatorInterface::setOLEDblink(int timeOn, int timeOff, bool blinkOn) {
    _oledBlink = blinkOn;
    _blinkTimeOn = timeOn;
    _blinkTimeOff = timeOff;
    _lastBlinkTime = millis();
    _blinkState = true;
    
    if (blinkOn) {
        _wakeOLED();
    }
}

void IndicatorInterface::setOLEDOff() {
    _oledOn = false;
    _oledBlink = false;
    u8g2.setPowerSave(1);
}

void IndicatorInterface::setOLEDOn() {
    _oledOn = true;
    _oledSleeping = false;
    u8g2.setPowerSave(0);
    _updateOLEDDisplay();
    _wakeOLED();
}

void IndicatorInterface::updateOLED() {
    if (!_oledOn) return;
    
    _handleOLEDSleep();
    if (_oledSleeping) return;
    
    // Handle special blinking first - if active, skip other operations
    if (_isBlinkingOK || _isBlinkingCross) {
        _handleSpecialBlink();
        return;  // Don't handle regular blink/scroll when special blinking
    }
    
    _handleOLEDBlink();
    _handleScrolling();
}

void IndicatorInterface::update() {
    updateBlinking();
    updateOLED();
}


// void IndicatorInterface::updateOLED() {
//     if (!_oledOn) return;
    
//     _handleOLEDSleep();
//     if (_oledSleeping) return;
    
//     _handleOLEDBlink();
//     _handleSpecialBlink();  // Add this line
//     _handleScrolling();
// }



void IndicatorInterface::_calculateDisplayParams() {
    int displayHeight = u8g2.getDisplayHeight(); // 64 pixels
    
    // Calculate optimal line height for each mode to use full display height
    switch (_oledLines) {
        case 1:
            // Use largest font and center it
            u8g2.setFont(u8g2_font_10x20_t_cyrillic);
            _lineHeight = displayHeight; // Full height for single line
            _charWidth = 10;
            _maxCharsPerLine = 12;
            break;
            
        case 2:
            // Use medium-large font
            //u8g2.setFont(u8g2_font_9x15_t_cyrillic);
            u8g2.setFont(u8g2_font_10x20_t_cyrillic);
            _lineHeight = displayHeight / 2; // 32px per line
            _charWidth = 9;
            _maxCharsPerLine = 14;
            break;
            
        case 3:
            // Use medium font
            //u8g2.setFont(u8g2_font_7x13_t_cyrillic);
            // u8g2.setFont(u8g2_font_9x15_t_cyrillic);
            u8g2.setFont(u8g2_font_10x20_t_cyrillic);
            _lineHeight = displayHeight / 3; // ~21px per line
            _charWidth = 7;
            _maxCharsPerLine = 18;
            break;
            
        case 4:
            // Use smaller font
            //u8g2.setFont(u8g2_font_5x7_t_cyrillic);
            u8g2.setFont(u8g2_font_9x15_t_cyrillic);
            _lineHeight = displayHeight / 4; // 16px per line
            _charWidth = 5;
            _maxCharsPerLine = 25;
            break;
            
        case 5:
        default:
            // Use smallest font
            u8g2.setFont(u8g2_font_9x15_t_cyrillic);
            // u8g2.setFont(u8g2_font_4x6_t_cyrillic);
            _lineHeight = displayHeight / 5; // ~12px per line
            _charWidth = 4;
            _maxCharsPerLine = 32;
            break;
    }
}


// void IndicatorInterface::_calculateDisplayParams() {
//     // Set font based on number of lines
//     switch (_oledLines) {
//         case 1:
//             u8g2.setFont(u8g2_font_10x20_t_cyrillic);
//             _lineHeight = 24;
//             _charWidth = 10;
//             _maxCharsPerLine = 12;
//             break;
//         case 2:
//             u8g2.setFont(u8g2_font_9x15_t_cyrillic);
//             _lineHeight = 20;
//             _charWidth = 9;
//             _maxCharsPerLine = 14;
//             break;
//         case 3:
//             u8g2.setFont(u8g2_font_7x13_t_cyrillic);
//             _lineHeight = 16;
//             _charWidth = 7;
//             _maxCharsPerLine = 18;
//             break;
//         case 4:
//             u8g2.setFont(u8g2_font_5x7_t_cyrillic);
//             _lineHeight = 12;
//             _charWidth = 6;
//             _maxCharsPerLine = 21;
//             break;
//         case 5:
//         default:
//             u8g2.setFont(u8g2_font_4x6_t_cyrillic);
//             _lineHeight = 10;
//             _charWidth = 5;
//             _maxCharsPerLine = 25;
//             break;
//     }
// }

void IndicatorInterface::_updateOLEDDisplay() {
    if (!_oledOn || _oledSleeping) return;
    
    u8g2.clearBuffer();
    _calculateDisplayParams();
    
    // Set font position to top for consistent positioning
    u8g2.setFontPosTop();
    
    int displayHeight = u8g2.getDisplayHeight(); // 64 pixels
    
    for (int i = 0; i < _oledLines && i < _textBufferSize; i++) {
        int yPos;
        
        if (_oledLines == 1) {
            // Center single line vertically
            int fontHeight = u8g2.getFontAscent() - u8g2.getFontDescent();
            yPos = (displayHeight - fontHeight) / 2;
        } else {
            // Distribute lines evenly across display height
            yPos = (i * displayHeight) / _oledLines;
        }
        
        _drawTextLine(i, yPos);
    }
    
    u8g2.sendBuffer();
}


/**
 * @brief Draw a single text line with circular scrolling support
 * @param[in] lineIndex Index of the line to draw (0-4)
 * @param[in] yPos Y position on the display
 * @details Implements circular scrolling for text longer than display width.
 *          Text continuously loops with a 3-space separator between end and beginning.
 */
void IndicatorInterface::_drawTextLine(int lineIndex, int yPos) {
    if (lineIndex >= _textBufferSize) return;
    
    String text = _textBuffer[lineIndex];
    
    // Get text width in pixels (works correctly with UTF-8)
    int textWidthPixels = u8g2.getUTF8Width(text.c_str());
    int displayWidthPixels = u8g2.getDisplayWidth();
    
    if (textWidthPixels <= displayWidthPixels) {
        // Short text - display normally
        u8g2.drawUTF8(0, yPos, text.c_str());
    } else {
        // Long text - simple scrolling without clipping
        int offset = _scrollOffset[lineIndex];
        
        // Calculate position for smooth scrolling
        int xPos = -offset;
        
        // Draw the main text
        u8g2.drawUTF8(xPos, yPos, text.c_str());
        
        // Draw the text again to create circular effect
        // Only if the first instance has started scrolling off screen
        if (offset > 0) {
            int secondXPos = xPos + textWidthPixels + u8g2.getUTF8Width("   ");
            if (secondXPos < displayWidthPixels) {
                u8g2.drawUTF8(secondXPos, yPos, text.c_str());
            }
        }
    }
}





void IndicatorInterface::_handleOLEDSleep() {
    if (_oledSleepDelay < 0) return;  // Never sleep
    
    if (!_oledSleeping && (millis() - _lastActivityTime) > _oledSleepDelay) {
        _oledSleeping = true;
        u8g2.setPowerSave(1);
    }
}

void IndicatorInterface::_handleOLEDBlink() {
    if (!_oledBlink || _oledSleeping) return;
    
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - _lastBlinkTime;
    
    if (_blinkState && elapsed > _blinkTimeOn) {
        // Turn off
        u8g2.setPowerSave(1);
        _blinkState = false;
        _lastBlinkTime = currentTime;
    } else if (!_blinkState && elapsed > _blinkTimeOff) {
        // Turn on
        u8g2.setPowerSave(0);
        _updateOLEDDisplay();
        _blinkState = true;
        _lastBlinkTime = currentTime;
    }
}

/**
 * @brief Handle circular scrolling animation for all text lines
 * @details Updates scroll offsets for lines longer than display width.
 *          Implements seamless circular scrolling where text wraps around continuously.
 */
void IndicatorInterface::_handleScrolling() {
    if (millis() - _lastScrollTime < _scrollDelay) return;
    
    bool needsUpdate = false;
    
    for (int i = 0; i < _textBufferSize; i++) {
        String text = _textBuffer[i];
        
        // Get text width in pixels
        int textWidthPixels = u8g2.getUTF8Width(text.c_str());
        int displayWidthPixels = u8g2.getDisplayWidth();
        
        if (textWidthPixels > displayWidthPixels) {
            // Scroll by configured pixels each time for smooth movement
            _scrollOffset[i] += SCROLL_SPEED_PIXELS;
            
            // Calculate total width including separator "   " (3 spaces)
            int separatorWidth = u8g2.getUTF8Width("   ");
            int totalWidth = textWidthPixels + separatorWidth;
            
            // Wrap around when we've scrolled the full text + separator
            if (_scrollOffset[i] >= totalWidth) {
                _scrollOffset[i] = 0;  // Wrap back to beginning
            }
            needsUpdate = true;
        } else {
            // Only reset offset for short lines
            _scrollOffset[i] = 0;
        }
    }
    
    if (needsUpdate) {
        _lastScrollTime = millis();
        _updateOLEDDisplay();
    }
}



void IndicatorInterface::_wakeOLED() {
    if (_oledOn) {
        _lastActivityTime = millis();
        _oledSleeping = false;
        u8g2.setPowerSave(0);
    }
}

// Modify existing handleInterrupt() method to wake OLED:
void IndicatorInterface::handleInterrupt() {
    if (_interruptFlag) {
        _interruptFlag = false;
        _updateState();
        _wakeOLED();  // Wake OLED on any interrupt
    }
}

// Add this method to IndicatorInterface.cpp
// void IndicatorInterface::_fixSH1106Offset() {
//     // Set column start address to 2 for SH1106
//     u8g2.sendF("ca", 0x10 | 0, 0x10 | 2);  // Set lower and higher column start address
// }

void IndicatorInterface::pushLine(String newLine) {
    // Shift all lines down
    for (int i = 4; i > 0; i--) {
        _textBuffer[i] = _textBuffer[i-1];
    }
    
    // Add new line at the top
    _textBuffer[0] = newLine;
    
    // Ensure we have the right number of lines
    if (_textBufferSize < _oledLines) {
        _textBufferSize++;
    } else {
        _textBufferSize = _oledLines;
    }
    
    // Reset scroll offsets
    for (int i = 0; i < 5; i++) {
        _scrollOffset[i] = 0;
    }
    
    _updateOLEDDisplay();
    _wakeOLED();
}

void IndicatorInterface::displayOK() {
    // Remove this line: stopBlinking(); // Stop any current blinking
    
    u8g2.clearBuffer();
    
    // Use the largest available font for OK
    u8g2.setFont(u8g2_font_logisoso42_tf);
    u8g2.setFontPosCenter();
    
    // Calculate center position
    int displayWidth = u8g2.getDisplayWidth();
    int displayHeight = u8g2.getDisplayHeight();
    
    String okText = "OK";
    int textWidth = u8g2.getUTF8Width(okText.c_str());
    
    int x = (displayWidth - textWidth) / 2;
    int y = displayHeight / 2;
    
    // Draw OK in center
    u8g2.drawUTF8(x, y + 5, okText.c_str());
    
    // Add a border around OK
    u8g2.drawFrame(x - 5, y - u8g2.getFontAscent() / 2 - 5, 
                   textWidth + 10, u8g2.getFontAscent() + u8g2.getFontDescent() + 20);
    
    u8g2.sendBuffer();
    _wakeOLED();
}

void IndicatorInterface::displayCross() {
    u8g2.clearBuffer();
    
    int displayWidth = u8g2.getDisplayWidth();
    int displayHeight = u8g2.getDisplayHeight();
    
    // Calculate center and radius for circle
    int centerX = displayWidth / 2;
    int centerY = displayHeight / 2;
    int radius = min(displayWidth, displayHeight) / 2 - 4;
    
    // Draw thicker circle by drawing multiple concentric circles
    for (int r = radius; r > radius - 3; r--) {
        u8g2.drawCircle(centerX, centerY, r);
    }
    
    // Cross parameters - made smaller
    int crossSize = (radius * 5) / 10;  // Reduced from 70% to 50% of radius
    int thickness = 4;  // Cross arm thickness
    int halfThickness = thickness / 2;
    
    // Draw main diagonal (top-left to bottom-right)
    for (int i = -crossSize; i <= crossSize; i++) {
        int x1 = centerX + i - halfThickness;
        int y1 = centerY + i - halfThickness;
        
        // Draw small filled rectangles along the diagonal
        u8g2.drawBox(x1, y1, thickness, thickness);
    }
    
    // Draw anti-diagonal (top-right to bottom-left)
    for (int i = -crossSize; i <= crossSize; i++) {
        int x1 = centerX + i - halfThickness;
        int y1 = centerY - i - halfThickness;
        
        // Draw small filled rectangles along the anti-diagonal
        u8g2.drawBox(x1, y1, thickness, thickness);
    }
    
    u8g2.sendBuffer();
    _wakeOLED();
}

// void IndicatorInterface::displayCross() {
//     // Remove this line: stopBlinking(); // Stop any current blinking
    
//     u8g2.clearBuffer();
    
//     int displayWidth = u8g2.getDisplayWidth();
//     int displayHeight = u8g2.getDisplayHeight();
    
//     // Calculate center and radius for circle
//     int centerX = displayWidth / 2;
//     int centerY = displayHeight / 2;
//     int radius = min(displayWidth, displayHeight) / 2 - 4;
    
//     // Draw circle
//     u8g2.drawCircle(centerX, centerY, radius);
    
//     // Draw cross inside circle
//     int crossSize = radius - 8;
    
//     // Draw X (cross) - make it thicker
//     for (int offset = -1; offset <= 1; offset++) {
//         u8g2.drawLine(centerX - crossSize + offset, centerY - crossSize, 
//                       centerX + crossSize + offset, centerY + crossSize);
//         u8g2.drawLine(centerX + crossSize + offset, centerY - crossSize, 
//                       centerX - crossSize + offset, centerY + crossSize);
//         u8g2.drawLine(centerX - crossSize, centerY - crossSize + offset, 
//                       centerX + crossSize, centerY + crossSize + offset);
//         u8g2.drawLine(centerX + crossSize, centerY - crossSize + offset, 
//                       centerX - crossSize, centerY + crossSize + offset);
//     }
    
//     u8g2.sendBuffer();
//     _wakeOLED();
// }


void IndicatorInterface::blinkOK(int blinkDelay) {
    // Stop any regular OLED blinking first
    _oledBlink = false;
    
    _saveCurrentText();
    _isBlinkingOK = true;
    _isBlinkingCross = false;
    _blinkDelayTime = blinkDelay;
    _lastBlinkToggle = millis();
    _blinkShowSpecial = true;
    
    displayOK();  // Start with OK showing
}

void IndicatorInterface::blinkCross(int blinkDelay) {
    // Stop any regular OLED blinking first
    _oledBlink = false;
    
    _saveCurrentText();
    _isBlinkingOK = false;
    _isBlinkingCross = true;
    _blinkDelayTime = blinkDelay;
    _lastBlinkToggle = millis();
    _blinkShowSpecial = true;
    
    displayCross();  // Start with cross showing
}


void IndicatorInterface::stopBlinking() {
    _isBlinkingOK = false;
    _isBlinkingCross = false;
    _restoreCurrentText();
}

void IndicatorInterface::_saveCurrentText() {
    // Save current text buffer
    for (int i = 0; i < 5; i++) {
        _savedTextBuffer[i] = _textBuffer[i];
    }
    _savedTextBufferSize = _textBufferSize;
    _savedOledLines = _oledLines;
}

void IndicatorInterface::_restoreCurrentText() {
    // Restore saved text buffer
    for (int i = 0; i < 5; i++) {
        _textBuffer[i] = _savedTextBuffer[i];
    }
    _textBufferSize = _savedTextBufferSize;
    _oledLines = _savedOledLines;
    
    _updateOLEDDisplay();
}

void IndicatorInterface::_handleSpecialBlink() {
    if (!_isBlinkingOK && !_isBlinkingCross) return;
    
    unsigned long currentTime = millis();
    if (currentTime - _lastBlinkToggle >= _blinkDelayTime) {
        _blinkShowSpecial = !_blinkShowSpecial;
        _lastBlinkToggle = currentTime;
        
        if (_blinkShowSpecial) {
            // Show OK or Cross
            if (_isBlinkingOK) {
                displayOK();
            } else if (_isBlinkingCross) {
                displayCross();
            }
        } else {
            // Show original text
            _restoreCurrentText();
        }
    }
}



void IndicatorInterface::startBlinking(const std::string& portName, unsigned long onTime, unsigned long offTime) {
    LoggerManager::info("INDICATION", 
        String(portName.c_str()) + " start blinking");
    // Check if port is already blinking
    for (auto& blinkPort : _blinkingPorts) {
        if (blinkPort.portName == portName) {
            // Update existing blinking parameters
            blinkPort.onTime = onTime;
            blinkPort.offTime = offTime;
            blinkPort.isActive = true;
            return;
        }
    }
    
    // Add new blinking port
    BlinkingPort newBlink;
    newBlink.portName = portName;
    newBlink.onTime = onTime;
    newBlink.offTime = offTime;
    newBlink.lastToggleTime = millis();
    newBlink.currentState = true;  // Start with ON
    newBlink.isActive = true;
    
    _blinkingPorts.push_back(newBlink);
    
    // Set initial state to ON
    writePort(portName, true);
}

void IndicatorInterface::stopBlinking(const std::string& portName) {
    LoggerManager::info("INDICATION", 
        String(portName.c_str()) + " stop blinking");
    for (auto it = _blinkingPorts.begin(); it != _blinkingPorts.end(); ++it) {
        if (it->portName == portName) {
            it->isActive = false;
            // Turn off the port when stopping blink
            writePort(portName, false);
            _blinkingPorts.erase(it);
            return;
        }
    }
}

void IndicatorInterface::updateBlinking() {
    unsigned long currentTime = millis();
    
    for (auto& blinkPort : _blinkingPorts) {
        if (!blinkPort.isActive) continue;
        
        unsigned long elapsed = currentTime - blinkPort.lastToggleTime;
        unsigned long targetTime = blinkPort.currentState ? blinkPort.onTime : blinkPort.offTime;
        
        if (elapsed >= targetTime) {
            // Toggle state
            blinkPort.currentState = !blinkPort.currentState;
            blinkPort.lastToggleTime = currentTime;
            
            // Update hardware
            writePort(blinkPort.portName, blinkPort.currentState);
        }
    }
}

bool IndicatorInterface::isBlinking(const std::string& portName) {
    for (const auto& blinkPort : _blinkingPorts) {
        if (blinkPort.portName == portName && blinkPort.isActive) {
            return true;
        }
    }
    return false;
}
