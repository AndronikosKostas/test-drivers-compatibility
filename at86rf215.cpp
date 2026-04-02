#include "at86rf215.hpp"
#include "RF_RXTask.hpp"
#include "RF_TXTask.hpp"
#include "main.h"

namespace AT86RF215 {

At86rf215 transceiver = At86rf215(&hspi4);
TransceiverHandler transceiver_handler;




void At86rf215::hardwareInit(uint16_t ms_delay_before_power_on, uint16_t ms_delay_after_power_on) {
    // Enable the 5V
    vTaskDelay(pdMS_TO_TICKS(ms_delay_before_power_on));
    HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_SET);
    /// ENABLE THE RX SWITCH
    HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_RESET);
    /// Essential for the trx to be able to send and receive packets
    /// (If you have it HIGH from the CubeMX the trx will not be able to send)
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(20));
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(ms_delay_after_power_on));
}

void At86rf215::powerOff() {
    HAL_GPIO_WritePin(P5V_RF_EN_GPIO_Port, P5V_RF_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_RX_UHF_GPIO_Port, EN_RX_UHF_Pin, GPIO_PIN_SET);
    turnOffUHFRXAmp();
    turnOffUHFTXAmp();
}


void At86rf215::turnOffUHFTXAmp() {
    HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_SET);
}

void At86rf215::turnOnUHFTXAmp() {
    HAL_GPIO_WritePin(EN_PA_UHF_GPIO_Port, EN_PA_UHF_Pin, GPIO_PIN_RESET);
}


void At86rf215::turnOnUHFRXAmp() {
    HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_SET);
}

void At86rf215::turnOffUHFRXAmp() {
    HAL_GPIO_WritePin(EN_UHF_AMP_RX_GPIO_Port, EN_UHF_AMP_RX_Pin, GPIO_PIN_RESET);
}

void At86rf215::chip_reset(Error& error) {
// Chip reset
    spi_write_8(RegisterAddress::RF_RST, 0x07, error);
    if (error != NO_ERRORS) {
        return;
    }

    // Reset IRQ status registers
    spi_read_8(RegisterAddress::RF09_IRQS, error);
    if (error != NO_ERRORS) {
        return;
    }

    spi_read_8(RegisterAddress::BBC0_IRQS, error);
    if (error != NO_ERRORS) {
        return;
    }
    // Restores the current config settings
    setup(error);
}

void At86rf215::hardwareReset(Error& err) {
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(20));
    HAL_GPIO_WritePin(RF_RST_GPIO_Port, RF_RST_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(100));
    transceiver.chip_reset(err);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void At86rf215::print_state(uint8_t state, const char* caller) {
    switch (state) {
        case 2: {
            LOG_ERROR << "[STATE][" << caller << "]: TRXOFF";
            break;
        }
        case 3: {
            LOG_INFO << "[STATE][" << caller << "]: TXPREP";
            break;
        }
        case 4: {
            LOG_INFO << "[STATE][" << caller << "]: TX";
            break;
        }
        case 5: {
            LOG_INFO << "[STATE][" << caller << "]: RX";
            break;
        }
        case 6: {
            LOG_INFO << "[STATE][" << caller << "]: TRANSITION";
            break;
        }
        case 7: {
            LOG_ERROR << "[STATE][" << caller << "]: RF_RESET";
            break;
        }
        default: {
            LOG_ERROR << "[STATE][" << caller << "]: ERROR, " << static_cast<int>(state);
            break;
        }
    }
}

State At86rf215::get_state(Transceiver transceiver, Error& err) {
    uint8_t state;
    if (transceiver == RF09) {
        state = spi_read_8(RF09_STATE, err) & 0x07;
    } else if (transceiver == RF24) {
        state = spi_read_8(RF24_STATE, err) & 0x07;
    }

    if (err != NO_ERRORS) {
        return RF_INVALID;
    }
    err = Error::NO_ERRORS;

    if ((state >= 0x02) && (state <= 0x07)) {
        return static_cast<State>(state);
    } else {
        err = Error::UKNOWN_REQUESTED_STATE;
        return State::RF_INVALID;
    }
}

State At86rf215::get_state_while(Transceiver transceiver, Error& err) {
    uint8_t state;

    while (true) {
        // Read the state register
        if (transceiver == RF09) {
            state = spi_read_8(RF09_STATE, err) & 0x07;
        } else if (transceiver == RF24) {
            state = spi_read_8(RF24_STATE, err) & 0x07;
        }

        // If SPI communication failed, retry
        if (err != NO_ERRORS) {
            vTaskDelay(pdMS_TO_TICKS(1));  // Short delay before retry
            continue;
        }

        // If state is valid, return it
        if ((state >= 0x02) && (state <= 0x07)) {
            err = Error::NO_ERRORS;
            return static_cast<State>(state);
        }

        // If state is invalid, retry after a short delay
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void At86rf215::set_state(Transceiver transceiver, State state_cmd,
                          Error& err) {
    if (transceiver == RF09) {
        spi_write_8(RF09_CMD, state_cmd, err);
    } else if (transceiver == RF24) {
        spi_write_8(RF24_CMD, state_cmd, err);
    }
}

void At86rf215::set_state_safe(Transceiver transceiver, State state_cmd, Error& err) {
    State current_state = static_cast<State>(get_state(transceiver, err));

    if (err != NO_ERRORS) {
        return;
    }

    err = Error::NO_ERRORS;

    // Use the comprehensive state transition validation
    if (!isStateTransitionSafe(current_state, state_cmd)) {
        LOG_WARNING << "[SET_STATE] Invalid transition from " << static_cast<int>(current_state)
                 << " to " << static_cast<int>(state_cmd);
        err = WRONG_STATE_IMPLICATION;
        return;
    }

    // If transition is valid, proceed with the command
    if (transceiver == RF09) {
        spi_write_8(RF09_CMD, state_cmd, err);
    } else if (transceiver == RF24) {
        spi_write_8(RF24_CMD, state_cmd, err);
    }
}


bool At86rf215::setStateWithRetry(Transceiver transceiver, State target_state, Error& error, uint8_t max_retries) {

    auto waitForStateTransition = [&](State target_state_, uint32_t timeout_ms = 50) -> bool {
        uint32_t start_time = HAL_GetTick();
        State current_state;

        do {
            vTaskDelay(pdMS_TO_TICKS(1));  // Short delay to prevent busy loop
            current_state = get_state(RF09, error);

            // Check for timeout
            if (HAL_GetTick() - start_time > timeout_ms) {
                return false;
            }
        } while (current_state != target_state_);

        return true;
    };

    for (uint8_t attempt = 0; attempt < max_retries; attempt++) {
        State current_state = get_state(transceiver, error);

        if (current_state == target_state) {
            error = NO_ERRORS;
            return true;
        }

        // Handle invalid states with hardware reset
        if (current_state == RF_INVALID) {
            spi_write_8(RegisterAddress::RF_RST, 0x07, error);
            vTaskDelay(pdMS_TO_TICKS(20)); // Allow reset to complete
            continue;
        }

        // Attempt state transition
        set_state(transceiver, target_state, error);
        if (error != NO_ERRORS) {
            LOG_ERROR << "State transition failed, attempt " << (attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(10)); // Brief delay before retry
            continue;
        }

        // Verify the transition completed
        if (waitForStateTransition(target_state, 20)) {
            error = NO_ERRORS;
            return true;
        }
    }

    error = FAILED_CHANGING_STATE;
    return false;
}



void At86rf215::ensureRXModeUpdated(Error& error, bool print_active) {
    transceiver.set_state(RF09, RF_TRXOFF, error);
    if (error != NO_ERRORS) {
        return;
    }
    transceiver.set_state(RF09, RF_RX, error);
    if (error != NO_ERRORS) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    auto state = transceiver.get_state(RF09, error);
    if (state == State::RF_RX) {
        if (print_active) {
            LOG_DEBUG << "[RX ENSURE] Successfully reached RX";
        }
    }
    else {
        LOG_ERROR << "[RX ENSURE] Did not reached RX, state: " << static_cast<uint8_t>(state);
    }
}
void At86rf215::ensureRXModeUpdatedAfterTX(Error& error, bool print_active) {

    transceiver.set_state(RF09, RF_RX, error);
    if (error != NO_ERRORS) {
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    auto state = transceiver.get_state(RF09, error);
    if (state == State::RF_RX) {
        if (print_active) {
            LOG_DEBUG << "[RX ENSURE] Successfully reached RX";
        }
    }
    else {
        LOG_ERROR << "[RX ENSURE] Did not reached RX, state: " << static_cast<uint8_t>(state);
    }
}

void At86rf215::pingTransceiver(uint8_t retries, uint16_t ms_wait_between_retries, AT86RF215::Error& err) {
    int attempt = 0;
    while (attempt < retries) {
        auto result = transceiver.check_transceiver_connection(err);
        if (result.has_value()) {
            LOG_INFO << "[SPI CONNECTION ESTABLISHED]";
            return;
        }
        attempt++;
        if (attempt < retries) {
            vTaskDelay(pdMS_TO_TICKS(ms_wait_between_retries));
        }
    }
}

void At86rf215::setFrequency(uint32_t frequency_kHz, AT86RF215::Error& err) {

    // Set the RX frequency
    transceiver.freqSynthesizerConfig.setFrequency_FineResolution_CMN_1(frequency_kHz);

    // set the proper state to control the regs
    transceiver.ensureTXModeUpdated(err, false);
    if (err != NO_ERRORS) {
        return;
    }
    // configure the pll
    transceiver.configure_pll(RF09,
                              transceiver.freqSynthesizerConfig.channelCenterFrequency09,
                              transceiver.freqSynthesizerConfig.channelNumber09,
                              transceiver.freqSynthesizerConfig.channelMode09,
                              transceiver.freqSynthesizerConfig.loopBandwidth09,
                              transceiver.freqSynthesizerConfig.channelSpacing09,
                              err);
    if (err != NO_ERRORS) {
        return;
    }

    // the reset calls the set_up
    transceiver.chip_reset(err);
    if (err != NO_ERRORS) {
        return;
    }
}


void At86rf215::ensureTXModeUpdated(Error& error, bool print_active) {

    transceiver.set_state(RF09, RF_TRXOFF, error);
    if (error != NO_ERRORS) {
        return;
    }
    auto state = transceiver.get_state(RF09, error);
    if (state == State::RF_TRXOFF) {
        if (print_active) {
            LOG_DEBUG << "[TX ENSURE] Successfully reached TRXOFF";
        }
    }
}


bool At86rf215::isStateTransitionSafe(State current_state, State target_state) {
    // Updated valid transitions based on AT86RF215 datasheet
    static const StateTransition valid_transitions[] = {
        // TRXOFF can be reached from ANY state (key datasheet rule)
        {RF_SLEEP, RF_TRXOFF},           // Wake-up from SLEEP
        {RF_TXPREP, RF_TRXOFF},          // CMD=TRXOFF
        {RF_TX, RF_TRXOFF},              // CMD=TRXOFF
        {RF_RX, RF_TRXOFF},              // CMD=TRXOFF
        {RF_TRANSITION, RF_TRXOFF},      // CMD=TRXOFF
        {RF_RESET, RF_TRXOFF},           // Automatic after reset
        {RF_INVALID, RF_TRXOFF},         // Recovery transition
        {RF_TRXOFF, RF_TRXOFF},          // Stay in same state

        // TXPREP can be reached from any state EXCEPT sleep states
        {RF_TRXOFF, RF_TXPREP},          // CMD=TXPREP (most common)
        {RF_TX, RF_TXPREP},              // Automatic after frame transmission OR CMD=TXPREP
        {RF_RX, RF_TXPREP},              // Automatic after frame reception OR CMD=TXPREP
        {RF_TRANSITION, RF_TXPREP},      // CMD=TXPREP during transition
        {RF_RESET, RF_TXPREP},           // CMD=TXPREP after reset
        {RF_INVALID, RF_TXPREP},         // Recovery via TXPREP
        {RF_TXPREP, RF_TXPREP},          // Stay in same state

        // TX can only be reached from TXPREP (datasheet requirement)
        {RF_TXPREP, RF_TX},              // CMD=TX (normal path)
        {RF_TX, RF_TX},                  // Stay in same state

        // RX can be reached from TXPREP or directly from TRXOFF
        {RF_TXPREP, RF_RX},              // CMD=RX (normal path)
        {RF_RX, RF_RX},
        {RF_TRXOFF, RF_RX},

        // SLEEP can only be entered from TRXOFF
        {RF_TRXOFF, RF_SLEEP},           // CMD=SLEEP (only valid transition to SLEEP)

        // DEEP_SLEEP is entered automatically when both transceivers in SLEEP

        // RESET can be initiated from any state
        {RF_TRXOFF, RF_RESET},           // CMD=RESET
        {RF_TXPREP, RF_RESET},           // CMD=RESET
        {RF_TX, RF_RESET},               // CMD=RESET
        {RF_RX, RF_RESET},               // CMD=RESET
        {RF_SLEEP, RF_RESET},            // CMD=RESET
        {RF_TRANSITION, RF_RESET},       // CMD=RESET
        {RF_INVALID, RF_RESET},          // CMD=RESET for recovery

        // TRANSITION state handling (intermediate state)
        {RF_TRXOFF, RF_TRANSITION},      // During state transitions
        {RF_TXPREP, RF_TRANSITION},      // During state transitions
        {RF_TX, RF_TRANSITION},          // During state transitions
        {RF_RX, RF_TRANSITION},          // During state transitions
        {RF_SLEEP, RF_TRANSITION},       // During state transitions
        {RF_RESET, RF_TRANSITION},       // During state transitions
        {RF_TRANSITION, RF_TRANSITION},  // Stay in transition

        // NOP command can be sent from any state (no state change)
        {RF_TRXOFF, RF_NOP},
        {RF_TXPREP, RF_NOP},
        {RF_TX, RF_NOP},
        {RF_RX, RF_NOP},
        {RF_SLEEP, RF_NOP},
        {RF_TRANSITION, RF_NOP},
        {RF_RESET, RF_NOP},
        {RF_INVALID, RF_NOP}
    };

    constexpr size_t transition_count = sizeof(valid_transitions) / sizeof(valid_transitions[0]);

    // Simple linear search through the valid transitions
    for (size_t i = 0; i < transition_count; i++) {
        if (valid_transitions[i].from == current_state &&
            valid_transitions[i].to == target_state) {
            return true;
        }
    }

    return false; // Transition not found, therefore not valid
}



static constexpr MorseCodeMapping getMorse(char c) {
    switch (c) {
        // Letters (uppercase + lowercase)
        case 'A': case 'a': return { 0b01000000, 2 };  // .-
        case 'B': case 'b': return { 0b10000000, 4 };  // -...
        case 'C': case 'c': return { 0b10100000, 4 };  // -.-.
        case 'D': case 'd': return { 0b10000000, 3 };  // -..
        case 'E': case 'e': return { 0b00000000, 1 };  // .
        case 'F': case 'f': return { 0b00100000, 4 };  // ..-.
        case 'G': case 'g': return { 0b11000000, 3 };  // --.
        case 'H': case 'h': return { 0b00000000, 4 };  // ....
        case 'I': case 'i': return { 0b00000000, 2 };  // ..
        case 'J': case 'j': return { 0b01110000, 4 };  // .---
        case 'K': case 'k': return { 0b10100000, 3 };  // -.-
        case 'L': case 'l': return { 0b01000000, 4 };  // .-..
        case 'M': case 'm': return { 0b11000000, 2 };  // --
        case 'N': case 'n': return { 0b10000000, 2 };  // -.
        case 'O': case 'o': return { 0b11100000, 3 };  // ---
        case 'P': case 'p': return { 0b01100000, 4 };  // .--.
        case 'Q': case 'q': return { 0b11010000, 4 };  // --.-
        case 'R': case 'r': return { 0b01000000, 3 };  // .-.
        case 'S': case 's': return { 0b00000000, 3 };  // ...
        case 'T': case 't': return { 0b10000000, 1 };  // -
        case 'U': case 'u': return { 0b00100000, 3 };  // ..-
        case 'V': case 'v': return { 0b00010000, 4 };  // ...-
        case 'W': case 'w': return { 0b01100000, 3 };  // .--
        case 'X': case 'x': return { 0b10010000, 4 };  // -..-
        case 'Y': case 'y': return { 0b10110000, 4 };  // -.--
        case 'Z': case 'z': return { 0b11000000, 4 };  // --..

        // Digits
        case '0': return { 0b11111000, 5 };
        case '1': return { 0b01111000, 5 };
        case '2': return { 0b00111000, 5 };
        case '3': return { 0b00011000, 5 };
        case '4': return { 0b00001000, 5 };
        case '5': return { 0b00000000, 5 };
        case '6': return { 0b10000000, 5 };
        case '7': return { 0b11000000, 5 };
        case '8': return { 0b11100000, 5 };
        case '9': return { 0b11110000, 5 };

        // Punctuation
        case '.': return { 0b01010100, 6 };  // .-.-.-
        case ',': return { 0b11001100, 6 };  // --..--
        case '?': return { 0b00110000, 6 };  // ..--..
        case '\'': return { 0b01111000, 6 };  // .----.
        case '!': return { 0b10101100, 6 };  // -.-.--
        case '/': return { 0b10010000, 5 };   // -..-.
        case '(': return { 0b10110000, 5 };   // -.--.
        case ')': return { 0b10110100, 6 };   // -.--.-
        case '&': return { 0b01000000, 5 };   // .-...
        case ':': return { 0b11100000, 6 };   // ---...
        case ';': return { 0b10101000, 6 };   // -.-.-.
        case '=': return { 0b10001000, 5 };   // -...-
        case '+': return { 0b01010000, 5 };   // .-.-.
        case '-': return { 0b10000100, 6 };   // -....-
        case '_': return { 0b00110100, 6 };   // ..--.-
        case '"': return { 0b01001000, 6 };   // .-..-.
        case '$': return { 0b00010010, 8 };   // ...-..-
        case '@': return { 0b01101000, 6 };   // .--.-.

        default:
            return { 0, 0 };  // not found
    }
}


void At86rf215::transmitMorseCode(Transceiver transceiver, Error& err, float wpm, const char* sequence, uint16_t sequenceLen, bool spi_dma_active) {

    RegisterAddress iqfc0_reg = RF_IQIFC0;
    RegisterAddress pc_reg;
    RegisterAddress txfhl_reg;
    RegisterAddress txfll_reg;
    RegisterAddress txdaci_reg;
    RegisterAddress txdacq_reg;

    if (transceiver == RF09) {
        pc_reg = BBC0_PC;
        txfhl_reg = BBC0_TXFLH;
        txfll_reg = BBC0_TXFLL;
        txdaci_reg = RF09_TXDACI;
        txdacq_reg = RF09_TXDACQ;
    } else {
        pc_reg = BBC1_PC;
        txfhl_reg = BBC1_TXFLH;
        txfll_reg = BBC1_TXFLL;
        txdaci_reg = RF24_TXDACI;
        txdacq_reg = RF24_TXDACQ;
    }
    uint8_t iqfc0_val;
    if (spi_dma_active) {
        iqfc0_val = spi_read_8_dma(iqfc0_reg, err);
    }
    else {
        iqfc0_val = spi_read_8(iqfc0_reg, err);
    }
    if (err != AT86RF215::NO_ERRORS)
        return;
    uint8_t pc_val;
    if (spi_dma_active) {
       pc_val = spi_read_8_dma(pc_reg, err);
    }
    else {
        pc_val = spi_read_8(pc_reg, err);
    }
    if (err != AT86RF215::NO_ERRORS)
        return;
    uint8_t txfhl_val;
    if (spi_dma_active) {
        txfhl_val = spi_read_8_dma(txfhl_reg, err);
    }
    else {
        txfhl_val = spi_read_8(txfhl_reg, err);
    }
    if (err != AT86RF215::NO_ERRORS)
        return;
    uint8_t txfll_val;
    if (spi_dma_active) {
        txfll_val = spi_read_8_dma(txfll_reg, err);
    }
    else {
        txfll_val = spi_read_8(txfll_reg, err);
    }

    if (err != AT86RF215::NO_ERRORS)
        return;

    if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF ||
        (transceiver == RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ||
        (transceiver == RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09)) {
        // The respective baseband core is active. Transmit using CTX (continuous transmit)
        spi_write_8(pc_reg, pc_val | 0x80, err);       // CTX = 1
        if (err != AT86RF215::NO_ERRORS)
            return;
        spi_write_8(txfhl_reg, 0x07, err);
        if (err != AT86RF215::NO_ERRORS)
            return; // any length will do
        spi_write_8(txfll_reg, 0xFF, err);
        if (err != AT86RF215::NO_ERRORS)
            return;
    } else {
        // Transmit using only the radio. EEC needs to be temporarily turned off, in order to
        // be able to control TXPREP-TX transitions manually
        spi_write_8(iqfc0_reg,  iqfc0_val & 0xFE, err);
        if (err != AT86RF215::NO_ERRORS)
            return;
    }

    spi_write_8(txdaci_reg, 0x80 | 0x7E, err); // enable in-phase DAC overwrite with max amplitude
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(txdacq_reg, 0x80 | 0x3F, err); // enable quadrature-phase DAC overwrite with min amplitude
    if (err != AT86RF215::NO_ERRORS)
        return;

    setStateWithRetry(transceiver, State::RF_TXPREP, err);
    if (err != AT86RF215::NO_ERRORS)
        return;
    bool txprep = xSemaphoreTake(transceiver_handler.txprep, pdMS_TO_TICKS(250)) == pdTRUE;
    if (!txprep) {
        err = AT86RF215::TXPREP_TIMEOUT;
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    TickType_t lastWakeTime = xTaskGetTickCount();

    const auto timeUnit = static_cast<uint16_t>(1200 / wpm);
    for (uint16_t i = 0; i < sequenceLen; i++) {
        if (sequence[i] == ' ') { // large delay for word gaps
            vTaskDelayUntil(&lastWakeTime, 7*pdMS_TO_TICKS(timeUnit)); /// TODO maybe do that 8 time units
            continue;
        }
        MorseCodeMapping morseCodeMapping = getMorse(sequence[i]);
        if (morseCodeMapping.dotDashNum == 0) { // skip unknown characters
            err = AT86RF215::UNKNOWN_CHAR;
        }

        // transmit character
        for (uint8_t j = 0; j < morseCodeMapping.dotDashNum; j++) {

            setStateWithRetry(transceiver, State::RF_TX, err);
            if (err != AT86RF215::NO_ERRORS)
                return;
            if (morseCodeMapping.dotDashMapping & (0x80 >> j) == 0) { // dot
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(timeUnit));
            } else {                                             // dash
                vTaskDelayUntil(&lastWakeTime, 3*pdMS_TO_TICKS(timeUnit));
            }
            setStateWithRetry(transceiver, State::RF_TXPREP, err);
            while (TransceiverReady_flag != true) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            txprep = xSemaphoreTake(transceiver_handler.txprep, pdMS_TO_TICKS(200)) == pdTRUE;
            if (!txprep) {
                err = AT86RF215::TXPREP_TIMEOUT;
                return;
            }

            // delay between character elements
            if (j != morseCodeMapping.dotDashNum - 1) {
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(timeUnit));
            }
        }

        // Delay between characters (skip if next is a space)
        if (i != sequenceLen - 1 && sequence[i + 1] != ' ') {
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(3 * timeUnit));
        }
    }

    // restore original configuration
    setStateWithRetry(transceiver, State::RF_TRXOFF, err);
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(iqfc0_reg, iqfc0_val, err);
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(pc_reg, pc_val, err);
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(txfhl_reg, txfhl_val, err);
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(txfll_reg, txfll_val, err);
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(txdaci_reg, 0x80 | 0x7E, err); // disable in-phase DAC overwrite
    if (err != AT86RF215::NO_ERRORS)
        return;
    spi_write_8(txdacq_reg, 0x80 | 0x3F, err); // disable quadrature-phase DAC overwrite
}

void At86rf215::transmitMorseCodeEnhanced(Transceiver transceiver, Error& err, float wpm, const char* sequence, uint16_t sequenceLen, bool spi_dma_active) {
    if (COMMSParameters::COMMS_TVAC_ACTIVE) {
        vTaskDelay(pdMS_TO_TICKS(1000 * COMMSParameters::COMMS_CW_ACTIVE_SEC));
        At86rf215::turnOffUHFTXAmp();
        LOG_DEBUG << "[TX CW] FAKE ENDED";
        err = AT86RF215::NO_ERRORS;
        return;
    }
    RegisterAddress iqfc0_reg = RF_IQIFC0;
    RegisterAddress pc_reg;
    RegisterAddress txfhl_reg;
    RegisterAddress txfll_reg;
    RegisterAddress txdaci_reg;
    RegisterAddress txdacq_reg;

    if (transceiver == RF09) {
        pc_reg = BBC0_PC;
        txfhl_reg = BBC0_TXFLH;
        txfll_reg = BBC0_TXFLL;
        txdaci_reg = RF09_TXDACI;
        txdacq_reg = RF09_TXDACQ;
    } else {
        pc_reg = BBC1_PC;
        txfhl_reg = BBC1_TXFLH;
        txfll_reg = BBC1_TXFLL;
        txdaci_reg = RF24_TXDACI;
        txdacq_reg = RF24_TXDACQ;
    }

    // Store original register values
    uint8_t iqfc0_val = spi_dma_active ? spi_read_8_dma_smhr(iqfc0_reg, err) : spi_read_8(iqfc0_reg, err);
    if (err != AT86RF215::NO_ERRORS) return;

    uint8_t pc_val = spi_dma_active ? spi_read_8_dma_smhr(pc_reg, err) : spi_read_8(pc_reg, err);
    if (err != AT86RF215::NO_ERRORS) return;

    uint8_t txfhl_val = spi_dma_active ? spi_read_8_dma_smhr(txfhl_reg, err) : spi_read_8(txfhl_reg, err);
    if (err != AT86RF215::NO_ERRORS) return;

    uint8_t txfll_val = spi_dma_active ? spi_read_8_dma_smhr(txfll_reg, err) : spi_read_8(txfll_reg, err);
    if (err != AT86RF215::NO_ERRORS) return;

    // Configure for morse code transmission
    if (iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF ||
        (transceiver == RF09 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF24) ||
        (transceiver == RF24 && iqInterfaceConfig.chipMode == ChipMode::RF_MODE_BBRF09)) {
        spi_write_8(pc_reg, pc_val | 0x80, err);       // CTX = 1
        if (err != AT86RF215::NO_ERRORS) return;
        spi_write_8(txfhl_reg, 0x07, err);
        if (err != AT86RF215::NO_ERRORS) return;
        spi_write_8(txfll_reg, 0xFF, err);
        if (err != AT86RF215::NO_ERRORS) return;
    } else {
        spi_write_8(iqfc0_reg, iqfc0_val & 0xFE, err);
        if (err != AT86RF215::NO_ERRORS) return;
    }

    spi_write_8(txdaci_reg, 0x80 | 0x7E, err); // enable in-phase DAC overwrite with max amplitude
    if (err != AT86RF215::NO_ERRORS) return;
    spi_write_8(txdacq_reg, 0x80 | 0x3F, err); // enable quadrature-phase DAC overwrite with min amplitude
    if (err != AT86RF215::NO_ERRORS) return;

    // Initial setup - get to TXPREP state once
    if (!setStateWithRetry(transceiver, State::RF_TXPREP, err, 3)) {
        err = AT86RF215::FAILED_CHANGING_STATE;
        return;
    }

    // Wait for initial TXPREP readiness - ONLY ONCE at the beginning
    if (!waitForTransmissionReady(100, err)) {
        return; // err already set
    }

    TickType_t lastWakeTime = xTaskGetTickCount();

    const auto timeUnit = static_cast<uint16_t>(1200 / wpm);

    // Fast transition helper function
    auto fastTxPrepTransition = [&]() -> bool {
        if (!setStateWithRetry(transceiver, State::RF_TXPREP, err, 5)) {
            return false;
        }

        // Quick wait for ready flag with timeout
        uint32_t start_time = HAL_GetTick();
        while (TransceiverReady_flag != true) {
            if (HAL_GetTick() - start_time > 50) { // 50ms timeout instead of 200ms
                err = AT86RF215::TXPREP_TIMEOUT;
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return true;
    };

    // Main transmission loop
    for (uint16_t i = 0; i < sequenceLen; i++) {
        if (sequence[i] == ' ') { // large delay for word gaps
            vTaskDelayUntil(&lastWakeTime, 7*pdMS_TO_TICKS(timeUnit));
            continue;
        }

        MorseCodeMapping morseCodeMapping = getMorse(sequence[i]);
        if (morseCodeMapping.dotDashNum == 0) { // skip unknown characters
            err = AT86RF215::UNKNOWN_CHAR;
            continue;
        }

        // transmit character
        for (uint8_t j = 0; j < morseCodeMapping.dotDashNum; j++) {
            // TX state for transmission
            set_state_safe(transceiver, RF_TX, err);
            if (err != NO_ERRORS) {
                return;
            }
            if ((morseCodeMapping.dotDashMapping & (0x80 >> j)) == 0) { // dot
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(timeUnit));
            } else { // dash
                vTaskDelayUntil(&lastWakeTime, 3*pdMS_TO_TICKS(timeUnit));
            }

            // Quick transition back to TXPREP - OPTIMIZED
            if (!fastTxPrepTransition()) {
                return;
            }

            // delay between character elements
            if (j != morseCodeMapping.dotDashNum - 1) {
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(timeUnit));
            }
        }

        // Delay between characters (skip if next is a space)
        if (i != sequenceLen - 1 && sequence[i + 1] != ' ') {
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(3 * timeUnit));
        }
    }

    // restore original configuration
    set_state_safe(transceiver, State::RF_TRXOFF, err);
    if (err != NO_ERRORS)
        return;



    spi_write_8(iqfc0_reg, iqfc0_val, err);
    spi_write_8(pc_reg, pc_val, err);
    spi_write_8(txfhl_reg, txfhl_val, err);
    spi_write_8(txfll_reg, txfll_val, err);
    spi_write_8(txdaci_reg, 0x00, err); // disable in-phase DAC overwrite (0x00, not 0x80|0x7E)
    spi_write_8(txdacq_reg, 0x00, err); // disable quadrature-phase DAC overwrite (0x00, not 0x80|0x3F)
}

void At86rf215::transmitBasebandPacketsTx(Transceiver transceiver, uint8_t* packet, uint16_t length, Error& err) {
    if (tx_ongoing || rx_ongoing) {
        err = ONGOING_TRANSMISSION_RECEPTION;
        return;
    }

    set_state(transceiver, RF_TRXOFF, err);
    if (err != NO_ERRORS) {
        return;
    }

    RegisterAddress regtxflh = BBC0_TXFLH;
    RegisterAddress regtxfll = BBC0_TXFLL;
    RegisterAddress regfbtxs = BBC0_FBTXS;

    if (transceiver == RF24) {
        regtxflh = BBC1_TXFLH;
        regtxfll = BBC1_TXFLL;
        regfbtxs = BBC1_FBTXS;
    }

    /// write length to register
    spi_write_8(regtxfll, length & 0xFF, err);
    if (err != NO_ERRORS) {
        return;
    }
    spi_write_8(regtxflh, (length >> 8) & 0x07, err);
    if (err != NO_ERRORS) {
        return;
    }

    spi_block_write_8(regfbtxs, length, packet, err);
    if (err != NO_ERRORS) {
        return;
    }
    set_state(transceiver, RF_TXPREP, err);
    tx_ongoing = true;
    waitForTransmissionReady(100, err);
    if (err != NO_ERRORS) {
        tx_ongoing = false;
    }
}


void At86rf215::readRxBuf(uint8_t* buf, uint16_t buffer_size, Error& err) {
    if (buf == nullptr) {
        LOG_ERROR << "[RX] Buffer pointer is null.";
        err = NULL_BUFF;
        return;
    }
    for (size_t i = 0; i < buffer_size; i++) {
        buf[i] = spi_read_8(BBC0_FBRXS + i, err);

        if (err != NO_ERRORS) {
            LOG_ERROR << "[RX] Error in reading the rx buf at index " << i
                      << " | Error Code: " << static_cast<int>(err);
            break;
        }
    }

}


bool At86rf215::waitForTransmissionReady(uint32_t timeout_ms, Error& err) {

    // Quick semaphore check with reduced timeout
    bool txprep = xSemaphoreTake(transceiver_handler.txprep, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
    if (!txprep) {
        err = AT86RF215::TXPREP_TIMEOUT;
        return false;
    }

    err = NO_ERRORS;
    return true;
}


void At86rf215::spi_write_8(uint16_t address, uint8_t value, Error& err) const {
    const uint8_t MAX_RETRIES = 5;
    const uint32_t RETRY_DELAY_MS = 40; // Delay between retries in milliseconds

    uint8_t msg[3] = {
        static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)),
        static_cast<uint8_t>(address & 0xFF),
        value
    };

    for (uint8_t retry = 0; retry <= MAX_RETRIES; retry++) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_error = HAL_SPI_Transmit(hspi, msg, 3, SPI_TIMEOUT);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);

        if (hal_error == HAL_OK) {
            err = Error::NO_ERRORS;
            return;
        }

        // If this isn't the last retry, add a small delay
        if (retry < MAX_RETRIES) {
            HAL_Delay(RETRY_DELAY_MS);
        }
    }

    // All retries failed
    err = Error::FAILED_WRITING_TO_REGISTER;
}
uint8_t At86rf215::spi_read_8(uint16_t address, Error& err) {
    const uint8_t MAX_RETRIES = 5;
    const uint32_t RETRY_DELAY_MS = 40; // Delay between retries in milliseconds

    uint8_t msg[2] = {
        static_cast<uint8_t>((address >> 8) & 0x7F),
        static_cast<uint8_t>(address & 0xFF)
    };
    uint8_t response[3] = {0};

    for (uint8_t retry = 0; retry <= MAX_RETRIES; retry++) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef hal_error = HAL_SPI_TransmitReceive(hspi, msg, response, 3, SPI_TIMEOUT);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);

        if (hal_error == HAL_OK) {
            err = Error::NO_ERRORS;
            return response[2];
        }

        // If this isn't the last retry, add a small delay
        if (retry < MAX_RETRIES) {
            HAL_Delay(RETRY_DELAY_MS);
        }
    }

    // All retries failed
    err = Error::FAILED_READING_FROM_REGISTER;
    return 0;
}


void At86rf215::spi_block_write_8(uint16_t address, uint16_t n, uint8_t* value, Error& err) {
    uint8_t msg[2] = {static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)), static_cast<uint8_t>(address & 0xFF)};
    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);
    uint8_t hal_error = HAL_SPI_Transmit(hspi, msg, 2, SPI_TIMEOUT);
    hal_error = HAL_SPI_Transmit(hspi, value, n, SPI_TIMEOUT);

    if (hal_error != HAL_OK) {
        err = Error::FAILED_WRITING_TO_REGISTER;
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        return;
    }
    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
    err = Error::NO_ERRORS;
}

uint8_t* At86rf215::spi_block_read_8(uint16_t address, uint8_t n, uint8_t* response, Error& err) {
    uint8_t msg[2] = {static_cast<uint8_t>((address >> 8) & 0x7F), static_cast<uint8_t>(address & 0xFF)};

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);
    uint8_t hal_error = HAL_SPI_TransmitReceive(hspi, msg, response, n + 2,
                                                SPI_TIMEOUT);

    if (hal_error != HAL_OK) {
        err = Error::FAILED_READING_FROM_REGISTER;
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        return response;
    }

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
    err = NO_ERRORS;
    return response + 2;
}



void At86rf215::spi_write_8_dma(uint16_t address, uint8_t value, Error& err) {
    uint8_t msg[3] = {
        static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)),
        static_cast<uint8_t>(address & 0xFF),
        value
    };

    // Lambda function for waiting with timeout
    auto waitForDmaComplete = [&](volatile bool& complete_flag, uint32_t timeout_ms = 100) -> bool {
        uint32_t start_time = HAL_GetTick();

        do {
            vTaskDelay(pdMS_TO_TICKS(1));  // Short delay to prevent busy loop

            // Check for timeout
            if (HAL_GetTick() - start_time > timeout_ms) {
                return false;
            }
        } while (!complete_flag);

        return true;
    };

    // Clear the flag before starting
    dma_tx_complete = false;

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET); // slave select pin

    uint8_t hal_error = HAL_SPI_Transmit_DMA(hspi, msg, 3);
    if (hal_error != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::TRANSMIT_DMA_HAL_ERROR;
        return;
    }

    // Wait for DMA completion using lambda
    if (waitForDmaComplete(dma_tx_complete, 100)) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;
    } else {
        // Timeout occurred
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::WRITE_DMA_TIMEOUT;
    }
}

void At86rf215::spi_block_write_8_dma(uint16_t address, uint16_t n, uint8_t* value, Error& err) {
    uint8_t msg[2] = {
        static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)),
        static_cast<uint8_t>(address & 0xFF)
    };

    // Lambda function for waiting with timeout
    auto waitForDmaComplete = [&](volatile bool& complete_flag, uint32_t timeout_ms = 100) -> bool {
        uint32_t start_time = HAL_GetTick();

        do {
            vTaskDelay(pdMS_TO_TICKS(1));  // Short delay to prevent busy loop

            // Check for timeout
            if (HAL_GetTick() - start_time > timeout_ms) {
                return false;
            }
        } while (!complete_flag);

        return true;
    };

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET); // slave select pin

    // Clear flag and transmit address bytes
    dma_tx_complete = false;
    if (HAL_SPI_Transmit_DMA(hspi, msg, sizeof(msg)) != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::FAILED_WRITING_TO_REGISTER;
        return;
    }

    // Wait for address transmit completion
    if (!waitForDmaComplete(dma_tx_complete, 100)) {
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::WRITE_DMA_TIMEOUT;
        return;
    }

    // Clear flag and transmit data block
    dma_tx_complete = false;
    if (HAL_SPI_Transmit_DMA(hspi, value, n) != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::FAILED_WRITING_TO_REGISTER;
        return;
    }

    // Wait for data transmit completion
    if (!waitForDmaComplete(dma_tx_complete, 100)) {
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::WRITE_DMA_TIMEOUT;
        return;
    }

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
    err = Error::NO_ERRORS;
}

    void At86rf215::spi_block_write_8_dma_smhr(uint16_t address, uint16_t n, uint8_t* value, Error& err) {
    uint8_t msg[2] = {
        static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)),
        static_cast<uint8_t>(address & 0xFF)
    };

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET); // slave select pin

    // Transmit address bytes
    uint8_t hal_error = HAL_SPI_Transmit_DMA(hspi, msg, sizeof(msg));
    if (hal_error != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::TRANSMIT_DMA_HAL_ERROR;
        return;
    }

    // Wait for address transmit completion using semaphore (100ms timeout)
    if (xSemaphoreTake(transceiver_handler.write_cplt_dma, pdMS_TO_TICKS(100)) != pdTRUE) {
        // Timeout occurred
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::WRITE_DMA_TIMEOUT;
        return;
    }

    // Transmit data block
    hal_error = HAL_SPI_Transmit_DMA(hspi, value, n);
    if (hal_error != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::TRANSMIT_DMA_HAL_ERROR;
        return;
    }

    // Wait for data transmit completion using semaphore (100ms timeout)
    if (xSemaphoreTake(transceiver_handler.write_cplt_dma, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;
    } else {
        // Timeout occurred
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::WRITE_DMA_TIMEOUT;
    }
}



    uint8_t At86rf215::spi_read_8_dma(uint16_t address, Error& err) {
        uint8_t tx_buffer[3] = {
            static_cast<uint8_t>((address >> 8) & 0x7F),
            static_cast<uint8_t>(address & 0xFF),
            0x00
        };
        uint8_t rx_buffer[3] = {0};

        // Lambda function for waiting with timeout
        auto waitForDmaComplete = [&](volatile bool& complete_flag, uint32_t timeout_ms = 100) -> bool {
            uint32_t start_time = HAL_GetTick();

            while (!complete_flag) {
                vTaskDelay(pdMS_TO_TICKS(1));  // 1ms delay to prevent busy loop

                // Check for timeout
                if (HAL_GetTick() - start_time > timeout_ms) {
                    return false;
                }
            }
            return true;
        };

        // Clear the flag before starting
        dma_rx_complete = false;

        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);

        if (HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer, rx_buffer, 3) != HAL_OK) {
            HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
            err = Error::FAILED_READING_FROM_REGISTER;
            return 0;
        }

        // Wait for DMA completion
        if (waitForDmaComplete(dma_rx_complete, 100)) {
            HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
            err = Error::NO_ERRORS;
            return rx_buffer[2];
        } else {
            // Timeout occurred
            HAL_SPI_DMAStop(hspi);
            HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
            err = Error::READ_DMA_TIMEOUT;
            return 0;
        }
    }


uint8_t* At86rf215::spi_block_read_8_dma(uint16_t address, uint8_t n, uint8_t* response, Error& err) {
    uint8_t msg[2] = {
        static_cast<uint8_t>((address >> 8) & 0x7F), // Read command
        static_cast<uint8_t>(address & 0xFF)
    };

    // Assert NSS
    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);

    // Start DMA transmit+receive
    if (HAL_SPI_TransmitReceive_DMA(hspi, msg, response, n + 2) != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::FAILED_READING_FROM_REGISTER;
        return nullptr;
    }

    // Wait for DMA completion (semaphore version)
    bool completed = xSemaphoreTake(transceiver_handler.read_cplt_dma, pdMS_TO_TICKS(1000)) == pdTRUE;

    // Always deassert NSS
    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);

    if (!completed) {
        err = Error::READ_DMA_TIMEOUT;
        return nullptr;
    }

    err = Error::NO_ERRORS;
    return response + 2; // Skip first two response bytes (command echo)
}

uint8_t At86rf215::spi_read_8_dma_smhr(uint16_t address, Error& err) {
    uint8_t tx_buffer[3] = {
        static_cast<uint8_t>((address >> 8) & 0x7F),
        static_cast<uint8_t>(address & 0xFF),
        0x00
    };
    uint8_t rx_buffer[3] = {0};

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer, rx_buffer, 3) != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::FAILED_READING_FROM_REGISTER;
        return 0;
    }

    // Wait for DMA completion using semaphore (100ms timeout)
    if (xSemaphoreTake(transceiver_handler.read_cplt_dma, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;
        return rx_buffer[2];
    } else {
        // Timeout occurred
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::READ_DMA_TIMEOUT;
        return 0;
    }
}

void At86rf215::spi_write_8_dma_smhr(uint16_t address, uint8_t value, Error& err) {
    uint8_t msg[3] = {
        static_cast<uint8_t>(0x80 | ((address >> 8) & 0x7F)),
        static_cast<uint8_t>(address & 0xFF),
        value
    };

    HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_RESET); // slave select pin

    uint8_t hal_error = HAL_SPI_Transmit_DMA(hspi, msg, 3);
    if (hal_error != HAL_OK) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::TRANSMIT_DMA_HAL_ERROR;
        return;
    }

    // Wait for DMA completion using semaphore (100ms timeout)
    if (xSemaphoreTake(transceiver_handler.write_cplt_dma, pdMS_TO_TICKS(100)) == pdTRUE) {
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;
    } else {
        // Timeout occurred
        HAL_SPI_DMAStop(hspi);
        HAL_GPIO_WritePin(RF_SPI_SEL_GPIO_Port, RF_SPI_SEL_Pin, GPIO_PIN_SET);
        err = Error::WRITE_DMA_TIMEOUT;
    }
}

void At86rf215::set_pll_channel_spacing(Transceiver transceiver,
                                        uint8_t spacing, Error& err) {
    RegisterAddress regscs;

    if (transceiver == RF09) {
        regscs = RF09_CS;
    } else if (transceiver == RF24) {
        regscs = RF24_CS;
    }
    spi_write_8(regscs, spacing, err);
}

uint8_t At86rf215::get_pll_channel_spacing(Transceiver transceiver,
                                           Error& err) {
    RegisterAddress regscs;

    if (transceiver == RF09) {
        regscs = RF09_CS;
    } else if (transceiver == RF24) {
        regscs = RF24_CS;
    }
    return spi_read_8(regscs, err);
}

void At86rf215::set_pll_channel_frequency(Transceiver transceiver, uint16_t freq, Error& err) {
    RegisterAddress regcf0h;
    RegisterAddress regcf0l;

    if (transceiver == RF09) {
        regcf0h = RF09_CCF0H;
        regcf0l = RF09_CCF0L;
    } else if (transceiver == RF24) {
        regcf0h = RF24_CCF0H;
        regcf0l = RF24_CCF0L;
    }

    spi_write_8(regcf0l, freq & 0x00FF, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    spi_write_8(regcf0h, (freq & 0xFF00) >> 8, err);
}

uint16_t At86rf215::get_pll_channel_frequency(Transceiver transceiver, Error& err) {
    RegisterAddress regcf0h;
    RegisterAddress regcf0l;

    if (transceiver == RF09) {
        regcf0h = RF09_CCF0H;
        regcf0l = RF09_CCF0L;
    } else if (transceiver == RF24) {
        regcf0h = RF24_CCF0H;
        regcf0l = RF24_CCF0L;
    }

    uint16_t cf0h = spi_read_8(regcf0h, err) & 0xFF;
    if (err != Error::NO_ERRORS) {
        return 0;
    }
    uint16_t cf0l = spi_read_8(regcf0l, err) & 0xFF;

    return (cf0h << 8) | cf0l;
}

uint16_t At86rf215::get_pll_channel_number(Transceiver transceiver, Error& err) {
    RegisterAddress regcnh;
    RegisterAddress regcnl;

    if (transceiver == RF09) {
        regcnh = RF09_CNM;
        regcnl = RF09_CNL;
    } else if (transceiver == RF24) {
        regcnh = RF24_CNM;
        regcnl = RF24_CNL;
    }

    uint16_t cnh = spi_read_8(regcnh, err) & 0x0100;
    if (err != Error::NO_ERRORS) {
        return 0;
    }
    uint16_t cnl = spi_read_8(regcnl, err) & 0x00FF;

    return (cnh << 8) | cnl;
}

void At86rf215::set_pll_bw(PLLBandwidth bw, Error& err) {
    uint8_t reg_pll = spi_read_8(RF09_PLL, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    // Clear bits [5:4] and set new value
    reg_pll &= ~(0x3 << 4);                     // Clear bits [5:4] (0x3 << 4 = 0b0011 0000)
    reg_pll |= (static_cast<uint8_t>(bw) << 4); // Set new value for bits [5:4]

    if (err != Error::NO_ERRORS) {
        return;
    }
}

PLLBandwidth At86rf215::get_pll_bw(Error& err) {
    uint8_t bw = (spi_read_8(RF09_PLL, err) >> 4) & 0x03;
    if (err != Error::NO_ERRORS) {
        return PLLBandwidth::BWInvalid;
    }
    return static_cast<PLLBandwidth>(bw);
}

PLLState At86rf215::get_pll_state(Transceiver transceiver, Error& err) {
    RegisterAddress regpll;

    if (transceiver == RF09) {
        regpll = RF09_PLL;
    } else if (transceiver == RF24) {
        regpll = RF24_PLL;
    }

    uint8_t state = spi_read_8(regpll, err) & 0x01;
    return static_cast<PLLState>(state);
}



void At86rf215::configure_pll(Transceiver transceiver, uint16_t freq, uint8_t channel_number, PLLChannelMode channel_mode, PLLBandwidth bw, uint8_t channel_spacing, Error& err) {

    if (get_state(transceiver, err) != RF_TRXOFF) {
        err = Error::INVALID_STATE_FOR_OPERATION;
        return;
    }

    if ((transceiver == Transceiver::RF09 && channel_mode == PLLChannelMode::FineResolution2443) || (transceiver == Transceiver::RF24 && channel_mode == PLLChannelMode::FineResolution450) || (transceiver == Transceiver::RF24 && channel_mode == PLLChannelMode::FineResolution900)) {
        err = Error::INVALID_TRANSCEIVER_FREQ;
        return;
    }

    uint32_t n_chan = (static_cast<uint32_t>(freq) << 8) | (channel_number & 0xFF);

    if ((((channel_mode == PLLChannelMode::FineResolution450) || (channel_mode == PLLChannelMode::FineResolution900)) && (n_chan < 126030 || n_chan > 1340967)) || ((channel_mode == PLLChannelMode::FineResolution2443) && (n_chan < 85700 || n_chan > 296172))) {
        err = Error::INVALID_TRANSCEIVER_FREQ;
        return;
    }
    /// RFn_CS
    set_pll_channel_spacing(transceiver, channel_spacing, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// RFn_CCF0L, RFn_CCF0H
    set_pll_channel_frequency(transceiver, freq, err);
    if (err != Error::NO_ERRORS) {
        return;
    }

    RegisterAddress reg_cnm;
    RegisterAddress reg_cnl;
    /// RFn_CNL
    if (transceiver == RF09) {
        reg_cnm = RF09_CNM;
        reg_cnl = RF09_CNL;
    } else if (transceiver == RF24) {
        reg_cnm = RF24_CNM;
        reg_cnl = RF24_CNL;
    }

    spi_write_8(reg_cnl, channel_number, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    // Modify only bits [7:6] of reg_cnm (the CNH is not needed for RF09)
    uint8_t reg_value_cm = spi_read_8(reg_cnm, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// RFn_CNM
    // Clear bits [7:6] and set the new value
    reg_value_cm &= ~(0x3 << 6);                               // Clear bits [7:6] (0x3 << 6 = 0b1100 0000)
    reg_value_cm |= (static_cast<uint8_t>(channel_mode) << 6); // Set new value for bits [7:6]

    // Write back the updated value
    spi_write_8(reg_cnm, reg_value_cm, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// RFn_PLL
    set_pll_bw(bw, err);
}

DevicePartNumber At86rf215::get_part_number(Error& err) {
    uint8_t dpn = spi_read_8_dma_smhr(RegisterAddress::RF_PN, err);
    if (err != Error::NO_ERRORS) {
        return DevicePartNumber::AT86RF215_INVALID;
    }

    if ((dpn >= 0x34) && (dpn <= 0x36)) {
        return static_cast<DevicePartNumber>(dpn);
    } else {
        err = Error::UKNOWN_PART_NUMBER;
        return DevicePartNumber::AT86RF215_INVALID;
    }
}

uint8_t At86rf215::get_version_number(Error& err) {
    uint8_t vn = spi_read_8_dma_smhr(RegisterAddress::RF_VN, err);
    if (err != Error::NO_ERRORS) {
        return 0;
    }
    return vn;
}

uint8_t At86rf215::get_pll_frequency(Transceiver transceiver, Error& err) {
    RegisterAddress regpll;

    if (transceiver == RF09) {
        regpll = RF09_PLLCF;
    } else if (transceiver == RF24) {
        regpll = RF24_PLLCF;
    }

    uint8_t freq = spi_read_8_dma_smhr(regpll, err) & 0x3F;
    if (err != Error::NO_ERRORS) {
        return 0;
    }
    return freq;
}

void At86rf215::set_tcxo_trimming(CrystalTrim trim, Error& err) {
    uint8_t trgxcov = spi_read_8_dma_smhr(RF_XOC, err) & 0x1F;
    if (err != Error::NO_ERRORS)
        return;
    spi_write_8(RF_XOC, (trgxcov & 0x10) | (static_cast<uint8_t>(trim) & 0x0F), err);
}

CrystalTrim At86rf215::read_tcxo_trimming(Error& err) {
    CrystalTrim regxoc =
        static_cast<CrystalTrim>(spi_read_8(RF_XOC, err) & 0x1F);
    if (err != Error::NO_ERRORS) {
        return CrystalTrim::TRIM_INV;
    }
    return regxoc;
}

void At86rf215::set_tcxo_fast_start_up_enable(bool fast_start_up, Error& err) {
    uint8_t trgxcov = spi_read_8_dma_smhr(RF_XOC, err) & 0x1F;
    if (err != Error::NO_ERRORS)
        return;
    spi_write_8(RF_XOC, (trgxcov & 0x0F) | (fast_start_up << 4), err);
}

bool At86rf215::read_tcxo_fast_start_up_enable(Error& err) {
    bool fast_start_up =
        static_cast<bool>((spi_read_8_dma_smhr(RF_XOC, err) & 0x10) >> 4);
    return fast_start_up;
}

PowerAmplifierRampTime At86rf215::get_pa_ramp_up_time(Transceiver transceiver, Error& err) {
    RegisterAddress regtxcutc;

    if (transceiver == RF09) {
        regtxcutc = RF09_TXCUTC;
    } else if (transceiver == RF24) {
        regtxcutc = RF24_TXCUTC;
    }

    uint8_t ramp = spi_read_8_dma_smhr(regtxcutc, err) & 0xC0 >> 6;
    return static_cast<PowerAmplifierRampTime>(ramp);
}

TransmitterCutOffFrequency At86rf215::get_cutoff_freq(Transceiver transceiver, Error& err) {
    RegisterAddress regtxcutc;

    if (transceiver == RF09) {
        regtxcutc = RF09_TXCUTC;
    } else if (transceiver == RF24) {
        regtxcutc = RF24_TXCUTC;
    }

    uint8_t cutoff = spi_read_8(regtxcutc, err) & 0x0F;
    return static_cast<TransmitterCutOffFrequency>(cutoff);
}


TxRelativeCutoffFrequency At86rf215::get_relative_cutoff_freq(Transceiver transceiver, Error& err) {
    RegisterAddress regtxdfe;

    if (transceiver == RF09) {
        regtxdfe = RF09_TXDFE;
    } else if (transceiver == RF24) {
        regtxdfe = RF24_TXDFE;
    }

    uint8_t dfe = (spi_read_8(regtxdfe, err) & 0x30) >> 5;
    return static_cast<TxRelativeCutoffFrequency>(dfe);
}


bool At86rf215::get_direct_modulation(Transceiver transceiver, Error& err) {
    RegisterAddress regtxdfe;

    if (transceiver == RF09) {
        regtxdfe = RF09_TXDFE;
    } else if (transceiver == RF24) {
        regtxdfe = RF24_TXDFE;
    }

    return (spi_read_8(regtxdfe, err) & 0x10) >> 4;
}


ReceiverSampleRate At86rf215::get_sample_rate(Transceiver transceiver, Error& err) {
    RegisterAddress regtxdfe;

    if (transceiver == RF09) {
        regtxdfe = RF09_TXDFE;
    } else if (transceiver == RF24) {
        regtxdfe = RF24_TXDFE;
    }

    return static_cast<ReceiverSampleRate>(spi_read_8(regtxdfe, err) & 0x1F);
}

PowerAmplifierCurrentControl At86rf215::get_pa_dc_current(
    Transceiver transceiver, Error& err) {
    RegisterAddress regpac;

    if (transceiver == RF09) {
        regpac = RF09_PAC;
    } else if (transceiver == RF24) {
        regpac = RF24_PAC;
    }

    uint8_t txpa = spi_read_8(regpac, err) & 0x60 >> 5;
    return static_cast<PowerAmplifierCurrentControl>(txpa);
}


bool At86rf215::get_lna_bypassed(Transceiver transceiver, Error& err) {
    RegisterAddress regaux =
        (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
    uint8_t lna_bypass = spi_read_8(regaux, err) & 0x80;
    if (err != Error::NO_ERRORS)
        return 0;
    return lna_bypass >> 7;
}

AutomaticGainControlMAP At86rf215::get_agcmap(Transceiver transceiver, Error& err) {
    RegisterAddress regaux =
        (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
    uint8_t agcmap = spi_read_8(regaux, err) & 0x60;
    if (err != Error::NO_ERRORS)
        return AutomaticGainControlMAP::AGC_INVALID;
    return static_cast<AutomaticGainControlMAP>(agcmap >> 5);
}


AutomaticVoltageExternal At86rf215::get_external_analog_voltage(
    Transceiver transceiver, Error& err) {
    RegisterAddress regaux =
        (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
    uint8_t agcmap = spi_read_8(regaux, err) & 0x10;
    if (err != Error::NO_ERRORS)
        return AutomaticVoltageExternal::INVALID;
    return static_cast<AutomaticVoltageExternal>(agcmap >> 4);
}

bool At86rf215::get_analog_voltage_settled_status(Transceiver transceiver, Error& err) {
    RegisterAddress regaux =
        (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
    uint8_t avs = spi_read_8(regaux, err) & 0x04;
    if (err != Error::NO_ERRORS)
        return 0;
    return avs >> 2;
}
PowerAmplifierVoltageControl At86rf215::get_analog_power_amplifier_voltage(Transceiver transceiver, Error& err) {
    RegisterAddress regaux =
        (transceiver == RF09) ? RegisterAddress::RF09_AUXS : RegisterAddress::RF24_AUXS;
    uint8_t pavc = spi_read_8(regaux, err) & 0x03;
    if (err != Error::NO_ERRORS)
        return PowerAmplifierVoltageControl::PAVC_INVALID;
    return static_cast<PowerAmplifierVoltageControl>(pavc);
}


void At86rf215::set_ed_average_detection(Transceiver transceiver, uint8_t df, EnergyDetectionTimeBasis dtb, Error& err) {
    RegisterAddress regedd;

    if (transceiver == RF09) {
        regedd = RF09_EDD;
    } else if (transceiver == RF24) {
        regedd = RF24_EDD;
    }

    uint8_t reg = ((df & 0xCF) << 2) | (static_cast<uint8_t>(dtb) & 0x3);
    spi_write_8(regedd, reg, err);
}

uint8_t At86rf215::get_ed_average_detection(Transceiver transceiver, Error& err) {
    RegisterAddress regedd;

    if (transceiver == RF09) {
        regedd = RF09_EDD;
    } else if (transceiver == RF24) {
        regedd = RF24_EDD;
    }

    uint8_t reg = spi_read_8(regedd, err);
    uint8_t df = (reg & 0xFC) >> 2;
    uint8_t dtb = (reg * 0x3);
    return df * dtb;
}

int8_t At86rf215::get_rssi(Transceiver transceiver, Error& err) {
    RegisterAddress regrssi;

    if (transceiver == RF09) {
        regrssi = RF09_RSSI;
    } else if (transceiver == RF24) {
        regrssi = RF24_RSSI;
    }

    int8_t reg = static_cast<int8_t>(spi_read_8(regrssi, err));

    if (err != Error::NO_ERRORS) {
        return 127;
    }
    if (reg > 4) {
        err = Error::INVALID_RSSI_MEASUREMENT;
        return 127;
    }
    return reg;
}


void At86rf215::set_battery_monitor_control(BatteryMonitorHighRange range, BatteryMonitorVoltageThreshold threshold, Error& err) {
    if (err != Error::NO_ERRORS) {
        return;
    }
    set_battery_monitor_high_range(range, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    set_battery_monitor_voltage_threshold(threshold, err);
}


BatteryMonitorStatus At86rf215::get_battery_monitor_status(Error& err) {
    uint8_t status = (spi_read_8(RF_BMDVC, err) & 0x20) >> 5;
    return static_cast<BatteryMonitorStatus>(status);
}

void At86rf215::set_battery_monitor_high_range(BatteryMonitorHighRange range,
                                               Error& err) {
    uint8_t bmhr = spi_read_8(RF_BMDVC, err) & 0x2F;
    if (err != Error::NO_ERRORS)
        return;
    spi_write_8(RF_BMDVC, (static_cast<uint8_t>(range) << 4) | bmhr, err);
}

uint8_t At86rf215::get_battery_monitor_high_range(Error& err) {
    return (spi_read_8(RF_BMDVC, err) & 0x10) >> 4;
}

void At86rf215::set_battery_monitor_voltage_threshold(
    BatteryMonitorVoltageThreshold threshold, Error& err) {
    uint8_t reg_value_bmvt = spi_read_8(RF_BMDVC, err);
    reg_value_bmvt &= ~(0xF);
    if (err != Error::NO_ERRORS)
        return;
    spi_write_8(RF_BMDVC, reg_value_bmvt | static_cast<uint8_t>(threshold), err);
}

uint8_t At86rf215::get_battery_monitor_voltage_threshold(Error& err) {
    return spi_read_8(RF_BMDVC, err) & 0x0F;
}

void At86rf215::set_external_front_end_control(Transceiver transceiver, ExternalFrontEndControl frontEndControl, Error& err) {
    RegisterAddress reg_address;
    if (transceiver == RF09)
        reg_address = RF09_PADFE;
    else if (transceiver == RF24)
        reg_address = RF24_PADFE;
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != Error::NO_ERRORS)
        return;
    // clears the bits [7:6]
    reg_value &= ~(0x3 << 6);
    reg_value |= static_cast<uint8_t>(frontEndControl) << 6;
    spi_write_8(reg_address, reg_value, err);
}

void At86rf215::setup_tx_frontend(Transceiver transceiver,
                                  PowerAmplifierRampTime pa_ramp_time, TransmitterCutOffFrequency cutoff,
                                  TxRelativeCutoffFrequency tx_rel_cutoff, Direct_Mod_Enable_FSKDM direct_mod,
                                  TransmitterSampleRate tx_sample_rate,
                                  PowerAmplifierCurrentControl pa_curr_control, uint8_t tx_out_power,
                                  ExternalLNABypass ext_lna_bypass, AutomaticGainControlMAP agc_map,
                                  AutomaticVoltageExternal avg_ext, AnalogVoltageEnable av_enable,
                                  PowerAmplifierVoltageControl pa_vcontrol, ExternalFrontEndControl externalFrontEndControl, Error& err) {
    RegisterAddress regtxcut;
    RegisterAddress regtxdfe;
    RegisterAddress regpac;
    RegisterAddress regauxs;

    uint8_t reg = 0;
    BBC0_FSKPE0;
    if (transceiver == Transceiver::RF09) {
        regtxcut = RF09_TXCUTC;
        regtxdfe = RF09_TXDFE;
        regpac = RF09_PAC;
        regauxs = RF09_AUXS;
    } else if (transceiver == Transceiver::RF24) {
        regtxcut = RF24_TXCUTC;
        regtxdfe = RF24_TXDFE;
        regpac = RF24_PAC;
        regauxs = RF24_AUXS;
    }
    /// Set RFn_TXCUTC
    reg = (static_cast<uint8_t>(pa_ramp_time) << 6) | static_cast<uint8_t>(cutoff);
    spi_write_8(regtxcut, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }

    /// Set RFn_TXDFE
    reg = (static_cast<uint8_t>(tx_rel_cutoff) << 5) | static_cast<uint8_t>(direct_mod) << 4 | static_cast<uint8_t>(tx_sample_rate);
    spi_write_8(regtxdfe, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// Set RFn_PAC
    reg = (static_cast<uint8_t>(pa_curr_control) << 5) | (tx_out_power & 0x1F);
    spi_write_8(regpac, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }

    /// Set RFn_AUXS
    reg = (static_cast<uint8_t>(ext_lna_bypass) << 7) | (static_cast<uint8_t>(agc_map) << 5) | (static_cast<uint8_t>(avg_ext) << 4) | (static_cast<uint8_t>(av_enable) << 3) | (static_cast<uint8_t>(pa_vcontrol));
    spi_write_8(regauxs, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// Set RFn_PADFE
    set_external_front_end_control(transceiver, externalFrontEndControl, err);
}

void At86rf215::setup_iq(ExternalLoopback external_loop,
                         IQOutputCurrent out_cur, IQmodeVoltage common_mode_vol,
                         IQmodeVoltageIEE common_mode_iee, EmbeddedControlTX embedded_tx_start,
                         ChipMode chip_mode, SkewAlignment skew_alignment, Error& err) {
    /// Set RF_IQIFC0
    uint8_t reg;
    reg = (static_cast<uint8_t>(external_loop) << 7) | (static_cast<uint8_t>(out_cur) << 4) | (static_cast<uint8_t>(common_mode_vol) << 2) | (static_cast<uint8_t>(common_mode_iee) << 1) | static_cast<uint8_t>(embedded_tx_start);
    spi_write_8(RF_IQIFC0, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }

    /// Set RF_IQIFC1
    reg = (static_cast<uint8_t>(chip_mode) << 4) | static_cast<uint8_t>(skew_alignment);
    spi_write_8(RF_IQIFC0, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
}

void At86rf215::setup_preamphasis(uint16_t fskpe0, uint16_t fskpe1,uint16_t fskpe2, Error& err){
    spi_write_8(BBC0_FSKPE0, fskpe0, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    spi_write_8(BBC0_FSKPE1, fskpe1, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    spi_write_8(BBC0_FSKPE2, fskpe2, err);
}


void At86rf215::setup_crystal(bool fast_start_up, CrystalTrim crystal_trim,
                              Error& err) {
    uint8_t reg = (static_cast<uint8_t>(fast_start_up) << 4) | static_cast<uint8_t>(crystal_trim);
    spi_write_8(RF_XOC, reg, err);
}

void At86rf215::setup_rx_energy_detection(Transceiver transceiver,
                                          EnergyDetectionMode energy_mode, uint8_t energy_detect_factor,
                                          EnergyDetectionTimeBasis energy_time_basis, Error& err) {
    uint8_t reg_value;
    RegisterAddress regedc;
    RegisterAddress regedd;

    if (transceiver == Transceiver::RF09) {
        regedc = RF09_EDC;
        regedd = RF09_EDD;
    } else if (transceiver == Transceiver::RF24) {
        regedc = RF24_EDC;
        regedd = RF24_EDD;
    }
    /// Read the current register value
    reg_value = spi_read_8(regedc, err);
    if (err != NO_ERRORS) {
        return;
    }

    /// Set RFn_EDC
    /// Clear bits [1:0] and update them with the new value
    reg_value &= ~0x03;                                    // Clear bits [1:0] (0x03 = 0000 0011)
    reg_value |= static_cast<uint8_t>(energy_mode) & 0x03; // Write new value to bits [1:0]
    spi_write_8(regedc, static_cast<uint8_t>(energy_mode), err);
    if (err != NO_ERRORS) {
        return;
    }
    /// Set RFn_EDD
    reg_value = 0;
    reg_value = (energy_detect_factor << 2) | static_cast<uint8_t>(energy_time_basis);
    spi_write_8(regedd, reg_value, err);
}

void At86rf215::setup_rx_frontend(Transceiver transceiver, bool if_inversion,
                                  bool if_shift, ReceiverBandwidth rx_bw,
                                  RxRelativeCutoffFrequency rx_rel_cutoff,
                                  ReceiverSampleRate rx_sample_rate, bool agc_input,
                                  AverageTimeNumberSamples agc_avg_sample, AGCReset agc_reset, AGCFreezeControl agc_freeze_control, AGCEnable agc_enable,
                                  AutomaticGainTarget agc_target, uint8_t gain_control_word, Error& err) {
    if (gain_control_word > 0x23) {
        err = INVALID_AGC_CONTROl_WORD;
        return;
    }

    RegisterAddress regrxbwc;
    RegisterAddress regrxdfe;
    RegisterAddress regagcc;
    RegisterAddress regagcs;

    uint8_t reg = 0;

    if (transceiver == Transceiver::RF09) {
        regrxbwc = RF09_RXBWC;
        regrxdfe = RF09_RXDFE;
        regagcc = RF09_AGCC;
        regagcs = RF09_AGCS;
    } else if (transceiver == Transceiver::RF24) {
        regrxbwc = RF24_RXBWC;
        regrxdfe = RF24_RXDFE;
        regagcc = RF24_AGCC;
        regagcs = RF24_AGCS;
    }

    /// Set RFn_RXBWC
    reg = spi_read_8(regrxbwc, err);
    reg = (reg & 0xC0) | (static_cast<uint8_t>(if_inversion) << 5) |
          (static_cast<uint8_t>(if_shift) << 4) |
          static_cast<uint8_t>(rx_bw);
    spi_write_8(regrxbwc, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// Set RFn_RXDFE
    reg = spi_read_8(regrxdfe, err);
    reg = (reg & 0x10) | (static_cast<uint8_t>(rx_rel_cutoff) << 5) | static_cast<uint8_t>(rx_sample_rate);
    spi_write_8(regrxdfe, reg, err);
    if (err != NO_ERRORS) {
        return;
    }

    /// Set RFn_AGGC
    reg = spi_read_8(regagcc, err);
    reg = (reg & (0x1 << 7)) | (static_cast<uint8_t>(agc_input) << 6) | (static_cast<uint8_t>(agc_avg_sample) << 4) | (static_cast<uint8_t>(agc_reset) << 3) | (static_cast<uint8_t>(agc_freeze_control) << 1) | (static_cast<uint8_t>(agc_enable) << 0);
    spi_write_8(regagcc, reg, err);
    if (err != Error::NO_ERRORS) {
        return;
    }
    /// Set RFn_AGCS
    if (agc_enable == AGCEnable::agc_disabled)
        reg = (static_cast<uint8_t>(agc_target) << 5) | gain_control_word;
    else {
        // Leave bits [4:0] unchanged, update bits [7:5]
        reg &= 0x1F;                                    // Mask out bits [7:5], keep [4:0] unchanged (0x1F = 00011111) because this value indicated the current receiver gain setting
        reg |= (static_cast<uint8_t>(agc_target) << 5); // Write to bits [7:5]
    }
    spi_write_8(regagcs, reg, err);
}

void At86rf215::setup_irq_cfg(bool maskMode, IRQPolarity polarity,
                              PadDriverStrength padDriverStrength, Error& err) {
    RegisterAddress regcfg = RF_CFG;
    uint8_t reg_value = spi_read_8_dma_smhr(regcfg, err);
    if (err != NO_ERRORS) {
        return;
    }
    reg_value &= ~(0xF);
    reg_value |= (maskMode << 3) | (static_cast<uint8_t>(polarity) << 2) | static_cast<uint8_t>(padDriverStrength);
    spi_write_8(regcfg, reg_value, err);
}

void At86rf215::setup_phy_baseband(Transceiver transceiver, bool continuousTransmit,
                                   bool frameSeqFilter, bool transmitterAutoFCS,
                                   FrameCheckSequenceType fcsType, bool basebandEnable,
                                   PhysicalLayerType phyType, Error& err) {
    RegisterAddress regphy;

    if (transceiver == Transceiver::RF09) {
        regphy = BBC0_PC;
    } else if (transceiver == Transceiver::RF24) {
        regphy = BBC1_PC;
    }

    spi_write_8(regphy,
                (continuousTransmit << 7) | (frameSeqFilter << 6) | (transmitterAutoFCS << 4) | (static_cast<uint8_t>(fcsType) << 3) | (basebandEnable << 2) | static_cast<uint8_t>(phyType),
                err);
}

    void At86rf215::setup_synch_word(Transceiver transceiver, uint16_t sfd0, uint16_t sfd1, Error& err) {
    RegisterAddress reg_sfd0_high;
    RegisterAddress reg_sfd0_low;
    RegisterAddress reg_sfd1_high;
    RegisterAddress reg_sfd1_low;

    // Select the correct register addresses based on the transceiver type
    if (transceiver == Transceiver::RF09) {
        reg_sfd0_high = BBC0_FSKSFD0H;
        reg_sfd0_low  = BBC0_FSKSFD0L;
        reg_sfd1_high = BBC0_FSKSFD1H;
        reg_sfd1_low  = BBC0_FSKSFD1L;
    } else if (transceiver == Transceiver::RF24) {
        reg_sfd0_high = BBC1_FSKSFD0H;
        reg_sfd0_low  = BBC1_FSKSFD0L;
        reg_sfd1_high = BBC1_FSKSFD1H;
        reg_sfd1_low  = BBC1_FSKSFD1L;
    } else {
        return;
    }

    // Write SFD0 low byte
    spi_write_8(reg_sfd0_low, static_cast<uint8_t>(sfd0 & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }

    // Write SFD0 high byte
    spi_write_8(reg_sfd0_high, static_cast<uint8_t>((sfd0 >> 8) & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }

    // Write SFD1 low byte
    spi_write_8(reg_sfd1_low, static_cast<uint8_t>(sfd1 & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }

    // Write SFD1 high byte
    spi_write_8(reg_sfd1_high, static_cast<uint8_t>((sfd1 >> 8) & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }
}


    void At86rf215::setup_rx_frame_buf_length(Transceiver transceiver, uint16_t length, Error& err) {
    RegisterAddress regrx_buf_length_high;
    RegisterAddress regrx_buf_length_low;

    // Select the correct register addresses based on the transceiver type
    if (transceiver == Transceiver::RF09) {
        regrx_buf_length_high = BBC0_FSKRRXFLH;
        regrx_buf_length_low  = BBC0_FSKRRXFLL;
    } else if (transceiver == Transceiver::RF24) {
        regrx_buf_length_high = BBC1_FSKRRXFLH;
        regrx_buf_length_low  = BBC1_FSKRRXFLL;
    } else {
        return;
    }

    // Write low byte of RX frame buffer length
    spi_write_8(regrx_buf_length_low, static_cast<uint8_t>(length & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }

    // Write high byte of RX frame buffer length
    spi_write_8(regrx_buf_length_high, static_cast<uint8_t>((length >> 8) & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }
}

    void At86rf215::setup_preamble_length(Transceiver transceiver, uint8_t length, Error& err) {
    RegisterAddress regrx_preamble_low;

    // Select the correct register addresses based on the transceiver type
    if (transceiver == Transceiver::RF09) {
        regrx_preamble_low  = BBC0_FSKPLL;
    } else if (transceiver == Transceiver::RF24) {

        regrx_preamble_low  = BBC1_FSKPLL;
    } else {
        return;
    }

    // Write low byte of RX frame buffer length
    spi_write_8(regrx_preamble_low, static_cast<uint8_t>(length & 0xFF), err);
    if (err != NO_ERRORS) {
        return;
    }
}




void At86rf215::setup_irq_mask(Transceiver transceiver, bool iqIfSynchronizationFailure, bool transceiverError,
                               bool batteryLow, bool energyDetectionCompletion, bool transceiverReady, bool wakeup,
                               bool frameBufferLevelIndication, bool agcRelease, bool agcHold,
                               bool transmitterFrameEnd, bool receiverExtendedMatch, bool receiverAddressMatch,
                               bool receiverFrameEnd, bool receiverFrameStart, Error& err) {
    RegisterAddress regbbc;
    RegisterAddress regrf;

    if (transceiver == Transceiver::RF09) {
        regbbc = BBC0_IRQM;
        regrf = RF09_IRQM;
    } else if (transceiver == Transceiver::RF24) {
        regbbc = BBC1_IRQM;
        regrf = RF24_IRQM;
    }

    spi_write_8(regrf, iqIfSynchronizationFailure << 5 | transceiverError << 4 | batteryLow << 3 | energyDetectionCompletion << 2 | transceiverReady << 1 | wakeup, err);
    if (err != NO_ERRORS) {
        return;
    }
    spi_write_8(regbbc, frameBufferLevelIndication << 7 | agcRelease << 6 | agcHold << 5 | transmitterFrameEnd << 4 | receiverExtendedMatch << 3 | receiverAddressMatch << 2 | receiverFrameEnd << 1 | receiverFrameStart, err);
}



etl::expected<void, Error> At86rf215::check_transceiver_connection(Error& err) {
    DevicePartNumber dpn = transceiver.get_part_number(err);
    if (err == NO_ERRORS && dpn == DevicePartNumber::AT86RF215) {
        return {}; /// success
    } else
        return etl::unexpected<Error>(err);
}

void At86rf215::set_bbc_fskc0_config(Transceiver transceiver,
                                     Bandwidth_time_product bt, Mod_index_scale midxs, Mod_index midx, FSK_mod_order mord,
                                     Error& err) {
    /// Define the appropriate register for BBCn_FSKC0 based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKC0; // Replace with actual RF09 register address
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKC0; // Replace with actual RF24 register address
    } else {
        return;
    }

    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return; // Return early if SPI read fails
    }
    reg_value &= 0x00; // Clear the bits that will be set explicitly

    /// Clear existing values in BT, MIDXS, MIDX, and MORD
    reg_value |= ((static_cast<uint8_t>(bt) & 0x03) << 6);    // BT: Bits [7:6]
    reg_value |= ((static_cast<uint8_t>(midxs) & 0x03) << 4); // MIDXS: Bits [5:4]
    reg_value |= ((static_cast<uint8_t>(midx) & 0x07) << 1);  // MIDX: Bits [3:1]
    reg_value |= (static_cast<uint8_t>(mord) & 0x01);         // MORD: Bit [0]

    /// Write the updated value back to the register
    spi_write_8(reg_address, reg_value, err);
}
void At86rf215::set_bbc_fskc1_config(Transceiver transceiver,
                                     Freq_Inversion freq_inv, MR_FSK_symbol_rate sr,
                                     Error& err) {
    // Define the appropriate register for BBCn_FSKC1 based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKC1; // Replace with actual RF09 register address
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKC1; // Replace with actual RF24 register address
    } else {
        return;
    }
    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return; // Return early if SPI read fails
    }
    // clear all bits except bit 4 (counting from 4)
    reg_value &= (0x1 << 4);
    reg_value |= (static_cast<uint8_t>(freq_inv) & 0x1) << 5;
    reg_value |= (static_cast<uint8_t>(sr) & 0xF);
    /// Write the updated value back to the register
    spi_write_8(reg_address, reg_value, err);
}
void At86rf215::set_bbc_fskc2_config(Transceiver transceiver, Preamble_Detection preamble_det,
                                     Receiver_Override rec_override,
                                     Receiver_Preamble_Timeout rec_preamble_timeout,
                                     Mode_Switch_Enable mode_switch_en,
                                     Preamble_Inversion preamble_inversion,
                                     FEC_Scheme fec_scheme,
                                     Interleaving_Enable interleaving_enable, Error& err) {
    /// Define the appropriate register for BBCn_FSKC2 based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKC2; // Replace with actual RF09 register address
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKC2; // Replace with actual RF24 register address
    } else {
        return;
    }
    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return; // Return early if SPI read fails
    }
    /// Update the register value with provided configurations
    reg_value &= 0x00; // Clear the bits that will be set explicitly
    /// Bit 7: PDTM - Preamble Detection Mode
    reg_value |= (static_cast<uint8_t>(preamble_det) & 0x1) << 7;
    /// Bits 6-5: RXO - Receiver Override
    reg_value |= (static_cast<uint8_t>(rec_override) & 0x3) << 5;
    /// Bit 4: RXPTO - Receiver Preamble Time Out
    reg_value |= (static_cast<uint8_t>(rec_preamble_timeout) & 0x1) << 4;
    /// Bit 3: MSE - Mode Switch Enable
    reg_value |= (static_cast<uint8_t>(mode_switch_en) & 0x1) << 3;
    /// Bit 2: PRI - Preamble Inversion
    reg_value |= (static_cast<uint8_t>(preamble_inversion) & 0x1) << 2;
    /// Bit 1: FECS - FEC Scheme
    reg_value |= (static_cast<uint8_t>(fec_scheme) & 0x1) << 1;
    /// Bit 0: FECIE - Interleaving Enable
    reg_value |= (static_cast<uint8_t>(interleaving_enable) & 0x1) << 0;
    /// Write the updated value back to the register
    spi_write_8(reg_address, reg_value, err);
}

void At86rf215::set_bbc_fskc3_config(Transceiver transceiver, SFD_Detection_Threshold sfdDetectionThreshold, Preamble_Detection_Threshold preambleDetectionThreshold, Error& err) {
    /// Define the appropriate register for BBCn_FSKC2 based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKC3; // Replace with actual RF09 register address
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKC3; // Replace with actual RF24 register address
    } else {
        return;
    }
    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return; // Return early if SPI read fails
    }
    /// Update the register value with provided configurations
    reg_value &= 0x00; // Clear the bits that will be set explicitly
    reg_value |= (static_cast<uint8_t>(sfdDetectionThreshold) & 0xF) << 4;
    reg_value |= (static_cast<uint8_t>(preambleDetectionThreshold) & 0xF) << 0;
    /// Write the updated value back to the register
    spi_write_8(reg_address, reg_value, err);
}

void At86rf215::set_bbc_fskc4_config(Transceiver transceiver,
                                     SFD_Quantization sfd_quantization,
                                     SFD_32 sfd_32,
                                     Raw_Mode_Reversal_Bit raw_mode_reversal,
                                     CSFD1 csfd1,
                                     CSFD0 csfd0,
                                     Error& err) {
    /// Define the appropriate register address for BBCn_FSKC4 based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKC4; // Replace with the actual RF09 register address
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKC4; // Replace with the actual RF24 register address
    } else {
        return; // Return early if the transceiver is invalid
    }

    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return; // Return early if SPI read fails
    }
    /// clear all bits except bit 7
    reg_value &= (0x01) << 7;
    reg_value |= (static_cast<uint8_t>(sfd_quantization) & 0x1) << 6;
    reg_value |= (static_cast<uint8_t>(sfd_32) & 0x1) << 5;
    reg_value |= (static_cast<uint8_t>(raw_mode_reversal) & 0x1) << 4;
    reg_value |= (static_cast<uint8_t>(csfd1) & 0x3) << 2;
    reg_value |= (static_cast<uint8_t>(csfd0) & 0x3) << 0;
    // Write the updated value back to the register
    spi_write_8(reg_address, reg_value, err);
}
void At86rf215::set_bbc_fskphrtx(Transceiver transceiver, SFD_Used sfdUsed, Data_Whitening dataWhitening, Error& err) {
    /// Define the appropriate register address for BBC0_FSKPHRTX based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKPHRTX; // Replace with the actual RF09 register address
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKPHRTX; // Replace with the actual RF24 register address
    } else {
        return; // Return early if the transceiver is invalid
    }

    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return; // Return early if SPI read fails
    }
    /// clear the bits to be updated
    /// 0000 1000 | 0000 0100 = 0000 1100 -> 1111 0011 -> reg_value = reg_value & 1111 0011
    reg_value &= ~((0x1 << 3) | (0x1 << 2));
    reg_value |= (static_cast<uint8_t>(sfdUsed) & 0x1) << 3;
    reg_value |= (static_cast<uint8_t>(dataWhitening) & 0x1) << 2;
    spi_write_8(reg_address, reg_value, err);
}

void At86rf215::set_bbc_fskdm(Transceiver transceiver, FSK_Preamphasis_Enable fskPreamphasisEnable, Direct_Mod_Enable_FSKDM directModEnableFskdm, Error& err) {
    /// Define the appropriate register address for BBCn_FSKDM based on the transceiver
    RegisterAddress reg_address;
    if (transceiver == RF09) {
        reg_address = BBC0_FSKDM;
    } else if (transceiver == RF24) {
        reg_address = BBC1_FSKDM;
    } else {
        return;
    }

    /// Read the current register value and mask out the fields to preserve other bits
    uint8_t reg_value = spi_read_8(reg_address, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// clear bits [1:0]
    reg_value &= ~((0x01 << 1) | (0x01 << 0));
    reg_value |= (static_cast<uint8_t>(fskPreamphasisEnable) & 0x1) << 1;
    reg_value |= (static_cast<uint8_t>(directModEnableFskdm) & 0x1) << 0;
    spi_write_8(reg_address, reg_value, err);
}

etl::expected<int16_t, Error> At86rf215::get_received_length(Transceiver transceiver, Error& err) {
    RegisterAddress reg_address_low;
    RegisterAddress reg_address_high;

    /// Determine the appropriate register addresses based on the transceiver
    if (transceiver == RF09) {
        reg_address_low = BBC0_RXFLL;
        reg_address_high = BBC0_RXFLH;
    } else {
        reg_address_low = BBC1_RXFLL;
        reg_address_high = BBC1_RXFLH;
    }
    uint8_t low_length_byte = spi_read_8(reg_address_low, err);
    if (err != NO_ERRORS) {
        return etl::unexpected<Error>(err);
    }

    /// Read the high-length byte
    uint8_t high_length_byte = spi_read_8(reg_address_high, err);
    if (err != NO_ERRORS) {
        return etl::unexpected<Error>(err); // Return the error
    }
    /// Combine the bytes to form the received length
    uint16_t received_length = (static_cast<uint16_t>(high_length_byte) << 8) | low_length_byte;

    return static_cast<int16_t>(received_length);
}


    void At86rf215::setup(Error& err) {
    /// Check state of RF09 core
    State state = get_state(RF09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// We have access to all registers only if we are in the state TRXOFF
    if (state != RF_TRXOFF) {
        err = INVALID_STATE_FOR_OPERATION;
        return;
    }
    /// Check state of RF24 core - we only proceed with the set-up if both cores are in the TRXOFF state to avoid setting half the registers
    state = get_state(RF24, err);
    if (err != NO_ERRORS) {
        return;
    }
    if (state != RF_TRXOFF) {
        err = INVALID_STATE_FOR_OPERATION;
        return;
    }

    /// Set IRQ masks
    setup_irq_mask(Transceiver::RF09, radioInterruptsConfig.iqIfSynchronizationFailure09, radioInterruptsConfig.transceiverError09,
                   radioInterruptsConfig.batteryLow09, radioInterruptsConfig.energyDetectionCompletion09, radioInterruptsConfig.transceiverReady09,
                   radioInterruptsConfig.wakeup09, interruptsConfig.frameBufferLevelIndication09, interruptsConfig.agcRelease09,
                   interruptsConfig.agcHold09, interruptsConfig.transmitterFrameEnd09, interruptsConfig.receiverExtendedMatch09,
                   interruptsConfig.receiverAddressMatch09, interruptsConfig.receiverFrameEnd09, interruptsConfig.receiverFrameStart09, err);

    setup_irq_mask(Transceiver::RF24, radioInterruptsConfig.iqIfSynchronizationFailure24, radioInterruptsConfig.transceiverError24,
                   radioInterruptsConfig.batteryLow24, radioInterruptsConfig.energyDetectionCompletion24, radioInterruptsConfig.transceiverReady24,
                   radioInterruptsConfig.wakeup24, interruptsConfig.frameBufferLevelIndication24, interruptsConfig.agcRelease24,
                   interruptsConfig.agcHold24, interruptsConfig.transmitterFrameEnd24, interruptsConfig.receiverExtendedMatch24,
                   interruptsConfig.receiverAddressMatch24, interruptsConfig.receiverFrameEnd24, interruptsConfig.receiverFrameStart24, err);

    /// Set IRQ pin
    setup_irq_cfg(generalConfig.irqMaskMode, generalConfig.irqPolarity,
                  generalConfig.padDriverStrength, err);

    /// Set PLL
    configure_pll(Transceiver::RF09, freqSynthesizerConfig.channelCenterFrequency09,
                  freqSynthesizerConfig.channelNumber09, freqSynthesizerConfig.channelMode09,
                  freqSynthesizerConfig.loopBandwidth09, freqSynthesizerConfig.channelSpacing09, err);
    if (err != NO_ERRORS) {
        return;
    }

    /// Setup Physical Layer for Baseband Cores
    setup_phy_baseband(Transceiver::RF09, basebandCoreConfig.continuousTransmit09,
                       basebandCoreConfig.frameCheckSequenceFilterEn09, basebandCoreConfig.transmitterAutoFrameCheckSequence09,
                       basebandCoreConfig.frameCheckSequenceType09, basebandCoreConfig.baseBandEnable09,
                       basebandCoreConfig.physicalLayerType09, err);
    if (err != NO_ERRORS) {
        return;
    }
    setup_phy_baseband(Transceiver::RF24, basebandCoreConfig.continuousTransmit24,
                       basebandCoreConfig.frameCheckSequenceFilterEn24, basebandCoreConfig.transmitterAutoFrameCheckSequence24,
                       basebandCoreConfig.frameCheckSequenceType24, basebandCoreConfig.baseBandEnable24,
                       basebandCoreConfig.physicalLayerType24, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBCn_FSKC0
    set_bbc_fskc0_config(RF09, basebandCoreConfig.bandwidth_time_09, basebandCoreConfig.midxs_09, basebandCoreConfig.midx_09, basebandCoreConfig.mord_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBCn_FSKC1
    set_bbc_fskc1_config(RF09, basebandCoreConfig.freq_inv_09, basebandCoreConfig.sr_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBCn_FSKC2
    set_bbc_fskc2_config(RF09, basebandCoreConfig.preamble_detection_09, basebandCoreConfig.receiver_override_09, basebandCoreConfig.receiver_preamble_timeout_09, basebandCoreConfig.mode_switch_en_09, basebandCoreConfig.preamble_inversion_09, basebandCoreConfig.fec_scheme_09, basebandCoreConfig.interleaving_enable_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBCn_FSKC3
    set_bbc_fskc3_config(RF09, basebandCoreConfig.sfdt_09, basebandCoreConfig.prdt_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBC_FSKC4
    set_bbc_fskc4_config(RF09, basebandCoreConfig.sfdQuantization_09, basebandCoreConfig.sfd32_09, basebandCoreConfig.rawModeReversalBit_09, basebandCoreConfig.csfd1_09, basebandCoreConfig.csfd0_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBCn_FSKPHRTX
    set_bbc_fskphrtx(RF09, basebandCoreConfig.sfdUsed_09, basebandCoreConfig.dataWhitening_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// BBCn_FSKDM
    set_bbc_fskdm(RF09, basebandCoreConfig.fskPreamphasisEnable_09, basebandCoreConfig.directModEnableFskdm_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// Set TX front-end
    setup_tx_frontend(Transceiver::RF09, txConfig.powerAmplifierRampTime09,
                      txConfig.transmitterCutOffFrequency09,
                      txConfig.txRelativeCutoffFrequency09, txConfig.directModulation09,
                      txConfig.transceiverSampleRate09,
                      txConfig.powerAmplifierCurrentControl09, txConfig.txOutPower09,
                      externalFrontEndConfig.externalLNABypass09, externalFrontEndConfig.automaticGainControlMAP09,
                      externalFrontEndConfig.automaticVoltageExternal09, externalFrontEndConfig.analogVoltageEnable09,
                      externalFrontEndConfig.powerAmplifierVoltageControl09, externalFrontEndConfig.externalFrontEnd_09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// Set up RX front-end
    setup_rx_frontend(Transceiver::RF09, rxConfig.ifInversion09, rxConfig.ifShift09,
                      rxConfig.receiverBandwidth09, rxConfig.rxRelativeCutoffFrequency09,
                      rxConfig.receiverSampleRate09, rxConfig.agcInput09,
                      rxConfig.averageTimeNumberSamples09, rxConfig.agcReset_09, rxConfig.agcFreezeControl_09, rxConfig.agcEnabled09,
                      rxConfig.automaticGainTarget09, rxConfig.gainControlWord09, err);
    if (err != NO_ERRORS) {
        return;
    }
    /// Set up IQ interface
    setup_iq(iqInterfaceConfig.externalLoopback, iqInterfaceConfig.iqOutputCurrent,
             iqInterfaceConfig.iqmodeVoltage, iqInterfaceConfig.iqmodeVoltageIEE,
             iqInterfaceConfig.embeddedControlTX, iqInterfaceConfig.chipMode, iqInterfaceConfig.skewAlignment,
             err);
    if (err != NO_ERRORS) {
        return;
    }

    /// Set up energy detection
    /// RFn_EDC, RFn_EDD
    setup_rx_energy_detection(Transceiver::RF09, rxConfig.energyDetectionMode09,
                              rxConfig.energyDetectDurationFactor09, rxConfig.energyDetectionBasis09, err);
    if (err != NO_ERRORS) {
        return;
    }

    /// Set up battery
    /// RF_BMDVC
    set_battery_monitor_control(generalConfig.batteryMonitorHighRange, generalConfig.batteryMonitorVoltage, err);
    if (err != NO_ERRORS) {
        return;
    }

    /// Set up crystal oscillator
    /// RF_XOC
    setup_crystal(generalConfig.fastStartUp, generalConfig.crystalTrim, err);

    //
    if (err != NO_ERRORS) {
        return;
    }
    setup_synch_word(RF09, basebandCoreConfig.sfd0, basebandCoreConfig.sfd1, err);
    // setup
    //
    if (err != NO_ERRORS) {
        return;
    }
    setup_rx_frame_buf_length(RF09, basebandCoreConfig.fixed_rx_length ,err);
    if (err != NO_ERRORS) {
        return;
    }
    setup_preamble_length(RF09, basebandCoreConfig.preambleLength, err);
    if (err != NO_ERRORS) {
        return;
    }
    // setup_preamphasis(2, 3, 252, err);
}



void At86rf215::handleIrq(Error& err) {
    err = NO_ERRORS;
    BaseType_t xHigherPriorityTaskWoken = false;
   /* Sub 1-GHz Transceiver */
    // Radio IRQ
    volatile uint8_t irq = spi_read_8_dma_smhr(RegisterAddress::RF09_IRQS, err);
    if (err != NO_ERRORS) {
        return;
    }
    if ((irq & InterruptMask::IFSynchronization) != 0) {
        IFSynchronization_flag = true;
    }
    if ((irq & InterruptMask::TransceiverError) != 0) {
        // Transceiver Error handling
        TransceiverError_flag = true;
    }
    if ((irq & InterruptMask::BatteryLow) != 0) {
        // Low Voltage
        Voltage_Drop = true;
    }
    if ((irq & InterruptMask::EnergyDetectionCompletion) != 0) {
        EnergyDetectionCompletion_flag = true;
        rx_ongoing = false;
        // cca_ongoing = false;
    }
    if ((irq & InterruptMask::TransceiverReady) != 0) {
        TransceiverReady_flag = true;
        xSemaphoreGive(transceiver_handler.txprep);

        if (tx_ongoing) {
            // Switch to TX state once the transceiver is ready to send
            if (get_state(RF09, err) != RF_TX)
                set_state(RF09, RF_TX, err);
        }
    }
    if ((irq & InterruptMask::Wakeup) != 0) {
        Wakeup_flag = true;
        // Wakeup handling
    }
    irq = spi_read_8_dma_smhr(RegisterAddress::BBC0_IRQS, err);
    if ((irq & InterruptMask::FrameBufferLevelIndication) != 0) {
        FrameBufferLevelIndication_flag = true;
    }
    if ((irq & InterruptMask::AGCRelease) != 0) {
        __NOP();
        // AGC Release handling
    }
    if ((irq & InterruptMask::AGCHold) != 0) {
        // AGC Hold handling
        COMMSParameters::COMMS_LAST_RSSI = transceiver.get_rssi(RF09, err);
        __NOP();

    }
    if ((irq & InterruptMask::TransmitterFrameEnd) != 0) {
        TransmitterFrameEnd_flag = true;
        tx_ongoing = false;
        xSemaphoreGive(transceiver_handler.txfeSemaphore_tx);
    }
    if ((irq & InterruptMask::ReceiverExtendMatch) != 0) {
        // Receiver Extended Match handling
        ReceiverExtendMatch_flag = true;
    }
    if ((irq & InterruptMask::ReceiverAddressMatch) != 0) {
        // Receiver Address Match handling
        ReceiverAddressMatch_flag = true;
    }
    if ((irq & InterruptMask::ReceiverFrameEnd) != 0) {
        ReceiverFrameEnd_flag = true;
        if (rx_ongoing)
            rx_ongoing = false;
        xTaskNotifyIndexed(rf_rxtask->taskHandle, NOTIFY_INDEX_RXFE_RX, RXFE_RX, eSetBits);
        xSemaphoreGive(transceiver_handler.rxfeSemaphore_tx);
    }
    if ((irq & InterruptMask::ReceiverFrameStart) != 0) {
        rx_ongoing = true;

    }
}


}