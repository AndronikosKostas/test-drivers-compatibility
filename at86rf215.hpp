#pragma once
#include <at86rf215definitions.hpp>
#include "at86rf215config.hpp"
#include "FreeRTOS.h"
#include "GlobalVariables.hpp"
#include "semphr.h"
#include <utility>
#include <cstdint>
#include <etl/expected.h>
#include "Logger.hpp"
#include "stm32h7xx_hal.h"

inline uint32_t SPI_TIMEOUT = 1000;
typedef __SPI_HandleTypeDef SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi4;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;

namespace AT86RF215 {

    class TransceiverHandler {
public:
    struct SemaphoreSet {
        SemaphoreHandle_t handle;
        StaticSemaphore_t buffer;
        const char* name;
    };

    // All semaphores organized
    SemaphoreSet task_level_resources_mtx_;
    SemaphoreSet spi_mtx_;
    SemaphoreSet hardware_access_mtx_;
    SemaphoreSet txfe_rx_sem_;
    SemaphoreSet txfe_tx_sem_;
    SemaphoreSet rxfe_rx_sem_;
    SemaphoreSet rxfe_tx_sem_;
    SemaphoreSet read_cplt_dma_sem_;
    SemaphoreSet write_cplt_dma_sem_;
    SemaphoreSet read_cplt_sem_;
    SemaphoreSet txprep_sem_;

    bool initialized_ = false;

    // Inline implementations to avoid linker issues
    inline bool createMutex(SemaphoreSet& semSet) {
        semSet.handle = xSemaphoreCreateMutexStatic(&semSet.buffer);
        if (semSet.handle == nullptr) {
            return false;
        }
        return true;
    }

    inline bool createBinarySemaphore(SemaphoreSet& semSet) {
        semSet.handle = xSemaphoreCreateBinaryStatic(&semSet.buffer);
        if (semSet.handle == nullptr) {
            return false;
        }
        return true;
    }

    // Public method to initialize semaphores (called by RF_ISR task)
    inline bool initializeAllSemaphores() {
        if (initialized_) {
            return true;
        }

        bool success = true;

        // Create mutex semaphores
        success &= createMutex(task_level_resources_mtx_);
        success &= createMutex(spi_mtx_);
        success &= createMutex(hardware_access_mtx_);

        // Create binary semaphores
        success &= createBinarySemaphore(txfe_rx_sem_);
        success &= createBinarySemaphore(txfe_tx_sem_);
        success &= createBinarySemaphore(rxfe_rx_sem_);
        success &= createBinarySemaphore(rxfe_tx_sem_);
        success &= createBinarySemaphore(read_cplt_dma_sem_);
        success &= createBinarySemaphore(write_cplt_dma_sem_);
        success &= createBinarySemaphore(read_cplt_sem_);
        success &= createBinarySemaphore(txprep_sem_);

        if (success) {
            // Update backward compatibility pointers
            task_level_resources_mtx = task_level_resources_mtx_.handle;
            spi_mtx = spi_mtx_.handle;
            hardware_access_mtx = hardware_access_mtx_.handle;
            txfeSemaphore_rx = txfe_rx_sem_.handle;
            txfeSemaphore_tx = txfe_tx_sem_.handle;
            rxfeSemaphore_rx = rxfe_rx_sem_.handle;
            rxfeSemaphore_tx = rxfe_tx_sem_.handle;
            read_cplt_dma = read_cplt_dma_sem_.handle;
            read_cplt = read_cplt_sem_.handle;
            write_cplt_dma = write_cplt_dma_sem_.handle;
            txprep = txprep_sem_.handle;

            initialized_ = true;
        } else {
            __NOP();
        }

        return success;
    }

    // Constructor does NOT initialize semaphores
    TransceiverHandler()
        : task_level_resources_mtx_{nullptr, {}, "task_level_resources_mtx"}
        , spi_mtx_{nullptr, {}, "spi_mtx"}
        , hardware_access_mtx_{nullptr, {}, "hardware_access_mtx"}
        , txfe_rx_sem_{nullptr, {}, "txfe_rx_semaphore"}
        , txfe_tx_sem_{nullptr, {}, "txfe_tx_semaphore"}
        , rxfe_rx_sem_{nullptr, {}, "rxfe_rx_semaphore"}
        , rxfe_tx_sem_{nullptr, {}, "rxfe_tx_semaphore"}
        , read_cplt_dma_sem_{nullptr, {}, "read_complete_dma_semaphore"}
        , write_cplt_dma_sem_{nullptr, {}, "write_complete_dma_semaphore"}
        , read_cplt_sem_{nullptr, {}, "read_complete_semaphore"}
        , txprep_sem_{nullptr, {}, "tx_prep_semaphore"}
    {
        // NO automatic initialization - will be done by RF_ISR task
    }

    ~TransceiverHandler() = default;

    bool isInitialized() const { return initialized_; }

    // Public accessors
    SemaphoreHandle_t getTaskLevelResourcesMutex() const { return task_level_resources_mtx_.handle; }
    SemaphoreHandle_t getSpiMutex() const { return spi_mtx_.handle; }
    SemaphoreHandle_t getHardwareAccessMutex() const { return hardware_access_mtx_.handle; }
    SemaphoreHandle_t getTxfeRxSemaphore() const { return txfe_rx_sem_.handle; }
    SemaphoreHandle_t getTxfeTxSemaphore() const { return txfe_tx_sem_.handle; }
    SemaphoreHandle_t getRxfeRxSemaphore() const { return rxfe_rx_sem_.handle; }
    SemaphoreHandle_t getRxfeTxSemaphore() const { return rxfe_tx_sem_.handle; }
    SemaphoreHandle_t getReadCompleteDmaSemaphore() const { return read_cplt_dma_sem_.handle; }
    SemaphoreHandle_t getWriteCompleteDmaSemaphore() const { return write_cplt_dma_sem_.handle; }
    SemaphoreHandle_t getReadCompleteSemaphore() const { return read_cplt_sem_.handle; }
    SemaphoreHandle_t getTxPrepSemaphore() const { return txprep_sem_.handle; }

    // Backward compatibility properties (can be removed later)
    SemaphoreHandle_t task_level_resources_mtx = nullptr;
    SemaphoreHandle_t spi_mtx = nullptr;
    SemaphoreHandle_t hardware_access_mtx = nullptr;
    SemaphoreHandle_t txfeSemaphore_rx = nullptr;
    SemaphoreHandle_t txfeSemaphore_tx = nullptr;
    SemaphoreHandle_t rxfeSemaphore_rx = nullptr;
    SemaphoreHandle_t rxfeSemaphore_tx = nullptr;
    SemaphoreHandle_t read_cplt_dma = nullptr;
    SemaphoreHandle_t read_cplt = nullptr;
    SemaphoreHandle_t write_cplt_dma = nullptr;
    SemaphoreHandle_t txprep = nullptr;
};

    typedef struct {
        uint8_t dotDashMapping;  // 0bXX represents the dot-dash mapping (e.g., 0b01 for dot-dash)
        uint8_t dotDashNum;      // The number of symbols in the Morse code
    } MorseCodeMapping;

    static constexpr MorseCodeMapping getMorse(char c);

    enum RF_STATE {
        // rx_ongoing = false, tx_ongoing = false
        READY = 0,
        // rx_ongoing = false, tx_ongoing = true
        TX_ONG = 1,
        // rx_ongoing = true, tx_ongoing = false
        RX_ONG = 2,
        // rx_ongoing = true, tx_ongoing = true
        RX_TX_ONG = 3,
    };


    inline uint8_t operator&(uint8_t a, InterruptMask b) {
        return a & static_cast<uint8_t>(b);
    }

    class At86rf215 {
    public:
        GeneralConfiguration generalConfig;
        RXConfig rxConfig;
        TXConfig txConfig;
        BasebandCoreConfig basebandCoreConfig;
        FrequencySynthesizer freqSynthesizerConfig;
        ExternalFrontEndConfig externalFrontEndConfig;
        InterruptsConfig interruptsConfig;
        RadioInterruptsConfig radioInterruptsConfig;
        IQInterfaceConfig iqInterfaceConfig;
        /// Flag indicating that a TX procedure is ongoing
        bool tx_ongoing;
        /// Flag indicating that an RX procedure is ongoing
        bool rx_ongoing;
        /// Flag indicating that the Clean Channel Assessment procedure is ongoing
        bool cca_ongoing;
        /// Flag for checking whether the AGC is locked
        bool agc_held;
        SPI_HandleTypeDef* hspi;
        uint32_t rx_frequency_ = COMMSParameters::COMMS_RF_RX_FREQUENCY;
        uint32_t tx_frequency_ = COMMSParameters::COMMS_RF_TX_FREQUENCY;

        transceiverHealth status = INIT;



        At86rf215(SPI_HandleTypeDef* hspim)
            : hspi(hspim),
              tx_ongoing(false), rx_ongoing(false), agc_held(false), cca_ongoing(false){}

        void setGeneralConfig(GeneralConfiguration&& GeneralConfig) {
            generalConfig = std::move(GeneralConfig);
        }
        void setRXConfig(RXConfig&& RXConfig) {
            rxConfig = std::move(RXConfig); // Move the new config into rxConfig
        }
        void setTXConfig(TXConfig&& TXConfig) {
            txConfig = std::move(TXConfig); // Move the new config into rxConfig
        }
        void setBaseBandCoreConfig(BasebandCoreConfig&& BasebandCoreConfig) {
            basebandCoreConfig = std::move(BasebandCoreConfig); // Move the new config into rxConfig
        }
        void setFrequencySynthesizerConfig(FrequencySynthesizer&& FrequencySynthesizer) {
            freqSynthesizerConfig = std::move(FrequencySynthesizer); // Move the new config into rxConfig
        }
        void setExternalFrontEndControlConfig(ExternalFrontEndConfig&& ExternalFrontEndConfig) {
            externalFrontEndConfig = std::move(ExternalFrontEndConfig);
        }
        void setInterruptConfig(InterruptsConfig&& InterruptsConfig) {
            interruptsConfig = std::move(InterruptsConfig);
        }
        void setRadioInterruptConfig(RadioInterruptsConfig&& RadioInterruptsConfig) {
            radioInterruptsConfig = std::move(RadioInterruptsConfig);
        }
        void setIQInterfaceConfig(IQInterfaceConfig&& IQInterfaceConfig) {
            iqInterfaceConfig = std::move(IQInterfaceConfig);
        }
        ///
        ///
        ///
        void initializeWithDefaults() {
            setGeneralConfig(GeneralConfiguration::DefaultGeneralConfig());
            setRXConfig(RXConfig::DefaultRXConfig());
            setTXConfig(TXConfig::DefaultTXConfig());
            setBaseBandCoreConfig(BasebandCoreConfig::DefaultBasebandCoreConfig(0x1ACFFC1D, FIXED_LENGTH_AT_SIZE_RX, 32));
            setFrequencySynthesizerConfig(FrequencySynthesizer::DefaultFrequencySynthesizerConfig());
            setExternalFrontEndControlConfig(ExternalFrontEndConfig::DefaultExternalFrontEndConfig());
            setInterruptConfig(InterruptsConfig::DefaultInterruptsConfig());
            setRadioInterruptConfig(RadioInterruptsConfig::DefaultRadioInterruptsConfig());
            setIQInterfaceConfig(IQInterfaceConfig::DefaultIQInterfaceConfig());
        }

        /// power functions
        static void hardwareInit(uint16_t ms_delay_before_power_on, uint16_t ms_delay_after_power_on);
        static void powerOff();
        static void turnOffUHFTXAmp();
        static void turnOnUHFTXAmp();
        static void turnOnUHFRXAmp();
        static void turnOffUHFRXAmp();

        /// reset functions
        void chip_reset(Error& error);
        static void hardwareReset(Error& error);

        /// control state functions
        State get_state(Transceiver transceiver, Error& err);
        State get_state_while(Transceiver transceiver, Error& err);
        void set_state(Transceiver transceiver, State state_cmd, Error& err);
        void set_state_safe(Transceiver transceiver, State state_cmd, Error& err);
        static void print_state(uint8_t state, const char* caller);
        bool setStateWithRetry(Transceiver transceiver, State target_state, Error& error, uint8_t max_retries = 3);

        /// validate state transitions
        struct StateTransition {
            State from;
            State to;

            bool operator==(const StateTransition& other) const {
                return (from == other.from) && (to == other.to);
            }

            bool operator<(const StateTransition& other) const {
                if (from != other.from) return from < other.from;
                return to < other.to;
            }
        };

        static bool isStateTransitionSafe(State current_state, State target_state);

        /// control between tx and rx basic states
        static void ensureTXModeUpdated(Error& error, bool print_active);
        static void ensureRXModeUpdated(Error& error, bool print_active);
        static void ensureRXModeUpdatedAfterTX(Error& error, bool print_active);

        /// receive
        void readRxBuf(uint8_t* buf, uint16_t buffer_size, Error& err);
        etl::expected<int16_t, Error> get_received_length(Transceiver transceiver, Error& err);

        /// transmit functions
        void transmitBasebandPacketsTx(Transceiver transceiver, uint8_t* packet, uint16_t length, Error& err);
        void transmitMorseCode(Transceiver transceiver, Error& err, float wpm, const char* sequence, uint16_t sequenceLen, bool spi_dma_active);
        void transmitMorseCodeEnhanced(Transceiver transceiver, Error& err, float wpm, const char* sequence, uint16_t sequenceLen, bool spi_dma_active);
        static bool waitForTransmissionReady(uint32_t timeout_ms, Error& err);

        /// SPI functions
        void spi_write_8(uint16_t address, uint8_t value, Error& err) const;
        uint8_t spi_read_8(uint16_t address, Error& err);
        void spi_block_write_8(uint16_t address, uint16_t n, uint8_t* value, Error& err);
        uint8_t* spi_block_read_8(uint16_t address, uint8_t n, uint8_t* response, Error& err);
        /// SPI with dma
        void spi_write_8_dma(uint16_t address, uint8_t value, Error& err);
        uint8_t spi_read_8_dma(uint16_t address, Error& err);
        void spi_block_write_8_dma(uint16_t address, uint16_t n, uint8_t* value, Error& err);
        uint8_t* spi_block_read_8_dma(uint16_t address, uint8_t n, uint8_t* response, Error& err);
        //
        uint8_t spi_read_8_dma_smhr(uint16_t address, Error& err);
        void spi_write_8_dma_smhr(uint16_t address, uint8_t value, Error& err);
        void spi_block_write_8_dma_smhr(uint16_t address, uint16_t n, uint8_t* value, Error& err);

        /// check SPI connection
        void pingTransceiver(uint8_t retries, uint16_t ms_wait_between_retries, AT86RF215::Error& err);
        etl::expected<void, Error> check_transceiver_connection(Error& err);
        DevicePartNumber get_part_number(Error& err);
        uint8_t get_version_number(Error& err);

        /// Frequency functions
        ///
        void setFrequency(uint32_t frequency_kHz, AT86RF215::Error& err);
        void set_pll_channel_spacing(Transceiver transceiver, uint8_t spacing,
                                     Error& err);

        uint8_t get_pll_channel_spacing(Transceiver transceiver, Error& err);


        void set_pll_channel_frequency(Transceiver transceiver, uint16_t freq,
                                       Error& err);


        uint16_t get_pll_channel_frequency(Transceiver transceiver, Error& err);

        uint16_t get_pll_channel_number(Transceiver transceiver, Error& err);


        void set_pll_bw(PLLBandwidth bw, Error& err);

        PLLBandwidth get_pll_bw(Error& err);

        PLLState get_pll_state(Transceiver transceiver, Error& err);


        void configure_pll(Transceiver transceiver, uint16_t freq,
                           uint8_t channel_number, PLLChannelMode channel_mode,
                           PLLBandwidth bw, uint8_t channel_spacing, Error& err);


        uint8_t get_pll_frequency(Transceiver transceiver, Error& err);

        void set_tcxo_trimming(CrystalTrim trim, Error& err);

        CrystalTrim read_tcxo_trimming(Error& err);


        void set_tcxo_fast_start_up_enable(bool fast_start_up, Error& err);


        bool read_tcxo_fast_start_up_enable(Error& err);

        ///
        PowerAmplifierRampTime get_pa_ramp_up_time(Transceiver transceiver,
                                                   Error& err);

        TransmitterCutOffFrequency get_cutoff_freq(Transceiver transceiver,
                                                   Error& err);

        TxRelativeCutoffFrequency get_relative_cutoff_freq(Transceiver transceiver,
                                                           Error& err);

        bool get_direct_modulation(Transceiver transceiver, Error& err);


        ReceiverSampleRate get_sample_rate(Transceiver transceiver, Error& err);


        PowerAmplifierCurrentControl get_pa_dc_current(Transceiver transceiver,
                                                       Error& err);

        bool get_lna_bypassed(Transceiver transceiver, Error& err);


        AutomaticGainControlMAP get_agcmap(Transceiver transceiver, Error& err);


        AutomaticVoltageExternal get_external_analog_voltage(
            Transceiver transceiver, Error& err);


        bool get_analog_voltage_settled_status(Transceiver transceiver, Error& err);


        PowerAmplifierVoltageControl get_analog_power_amplifier_voltage(
            Transceiver transceiver, Error& err);


        /// RSSI functions
        void set_ed_average_detection(Transceiver transceiver, uint8_t df,
                                      EnergyDetectionTimeBasis dtb, Error& err);

        uint8_t get_ed_average_detection(Transceiver transceiver, Error& err);

        int8_t get_rssi(Transceiver transceiver, Error& err);


        /// analog voltage - battery functions
        BatteryMonitorStatus get_battery_monitor_status(Error& err);

        void set_battery_monitor_high_range(BatteryMonitorHighRange range,
                                            Error& err);


        uint8_t get_battery_monitor_high_range(Error& err);

        void set_battery_monitor_voltage_threshold(BatteryMonitorVoltageThreshold threshold,
                                                   Error& err);
        void set_battery_monitor_control(BatteryMonitorHighRange range, BatteryMonitorVoltageThreshold threshold, Error& err);

        uint8_t get_battery_monitor_voltage_threshold(Error& err);

        ///
        void setup_tx_frontend(Transceiver transceiver,
                               PowerAmplifierRampTime pa_ramp_time,
                               TransmitterCutOffFrequency cutoff,
                               TxRelativeCutoffFrequency tx_rel_cutoff, Direct_Mod_Enable_FSKDM direct_mod,
                               TransmitterSampleRate tx_sample_rate,
                               PowerAmplifierCurrentControl pa_curr_control, uint8_t tx_out_power,
                               ExternalLNABypass ext_lna_bypass, AutomaticGainControlMAP agc_map,
                               AutomaticVoltageExternal avg_ext, AnalogVoltageEnable av_enable,
                               PowerAmplifierVoltageControl pa_vcontrol, ExternalFrontEndControl externalFrontEndControl, Error& err);


        void setup_rx_frontend(Transceiver transceiver, bool if_inversion,
                               bool if_shift, ReceiverBandwidth rx_bw,
                               RxRelativeCutoffFrequency rx_rel_cutoff,
                               ReceiverSampleRate rx_sample_rate, bool agc_input,
                               AverageTimeNumberSamples agc_avg_sample, AGCReset agc_reset, AGCFreezeControl agc_freeze_control, AGCEnable agc_enable,
                               AutomaticGainTarget agc_target, uint8_t gain_control_word, Error& err);

        void setup_rx_energy_detection(Transceiver transceiver, EnergyDetectionMode energy_mode,
                                       uint8_t energy_detect_factor,
                                       EnergyDetectionTimeBasis energy_time_basis, Error& err);


        void setup_crystal(bool fast_start_up, CrystalTrim crystal_trim,
                           Error& err);

        void setup_preamphasis(uint16_t fskpe0, uint16_t fskpe1,uint16_t fskpe2, Error& err);

        void setup_irq_cfg(bool maskMode, IRQPolarity irqPolarity,
                           PadDriverStrength padDriverStrength, Error& err);

        void setup_phy_baseband(Transceiver transceiver, bool continuousTransmit, bool frameSeqFilter, bool transmitterAutoFCS,
                                FrameCheckSequenceType fcsType, bool basebandEnable, PhysicalLayerType phyType, Error& err);

        void setup_synch_word(Transceiver transceiver, uint16_t sfd0, uint16_t sdf1, Error& err) ;

        void setup_rx_frame_buf_length(Transceiver transceiver, uint16_t length, Error& err);

        void setup_preamble_length(Transceiver transceiver, uint8_t length, Error& err);


        void set_bbc_fskc0_config(Transceiver transceiver,
                                  Bandwidth_time_product bt, Mod_index_scale midxs, Mod_index midx, FSK_mod_order mord,
                                  Error& err);

        void set_bbc_fskc1_config(Transceiver transceiver,
                                  Freq_Inversion freq_inv, MR_FSK_symbol_rate sr,
                                  Error& err);
        void set_bbc_fskc2_config(Transceiver transceiver, Preamble_Detection preamble_det,
                                  Receiver_Override rec_override,
                                  Receiver_Preamble_Timeout rec_preamble_timeout,
                                  Mode_Switch_Enable mode_switch_en,
                                  Preamble_Inversion preamble_inversion,
                                  FEC_Scheme fec_sheme,
                                  Interleaving_Enable interleaving_enable, Error& err);
        void set_bbc_fskc3_config(Transceiver transceiver, SFD_Detection_Threshold sfdDetectionThreshold,
                                  Preamble_Detection_Threshold preambleDetectionThreshold,
                                  Error& err);
        void set_bbc_fskc4_config(Transceiver transceiver,
                                  SFD_Quantization sfd_quantization,
                                  SFD_32 sfd_32,
                                  Raw_Mode_Reversal_Bit raw_mode_reversal,
                                  CSFD1 csfd1,
                                  CSFD0 csfd0,
                                  Error& err);
        void set_bbc_fskphrtx(Transceiver transceiver,
                              SFD_Used sfdUsed,
                              Data_Whitening dataWhitening,
                              Error& err);
        void set_bbc_fskdm(Transceiver transceiver,
                           FSK_Preamphasis_Enable fskPreamphasisEnable,
                           Direct_Mod_Enable_FSKDM directModEnableFskdm,
                           Error& err);

        void set_external_front_end_control(Transceiver transceiver,
                                            ExternalFrontEndControl frontEndControl,
                                            Error& err);

        void setup_iq(ExternalLoopback external_loop, IQOutputCurrent out_cur,
                      IQmodeVoltage common_mode_vol, IQmodeVoltageIEE common_mode_iee,
                      EmbeddedControlTX embedded_tx_start, ChipMode chip_mode,
                      SkewAlignment skew_alignment, Error& err);


        void setup_irq_mask(Transceiver transceiver, bool iqIfSynchronizationFailure, bool transceiverError,
                            bool batteryLow, bool energyDetectionCompletion, bool transceiverReady, bool wakeup,
                            bool frameBufferLevelIndication, bool agcRelease, bool agcHold,
                            bool transmitterFrameEnd, bool receiverExtendedMatch, bool receiverAddressMatch,
                            bool receiverFrameEnd, bool receiverFrameStart, Error& err);


        void setup(Error& err);

        void handleIrq(Error& err);

        /// Radio interrupts
        bool IFSynchronization_flag, TransceiverError_flag, EnergyDetectionCompletion_flag, TransceiverReady_flag, Wakeup_flag, Voltage_Drop = false;

        /// Baseband Core Interrupts
        bool FrameBufferLevelIndication_flag, AGCRelease_flag, AGCHold_flag, TransmitterFrameEnd_flag, ReceiverExtendMatch_flag, ReceiverAddressMatch_flag, ReceiverFrameEnd_flag, ReceiverFrameStart_flag = false;

        volatile bool dma_rx_complete = false;
        volatile bool dma_tx_complete = false;

    };

    extern At86rf215 transceiver;
    extern TransceiverHandler transceiver_handler;



} // namespace AT86RF215