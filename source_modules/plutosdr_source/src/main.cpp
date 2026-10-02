#include <utils/flog.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/style.h>
#include <gui/smgui.h>
#include <iio.h>
#include <ad9361.h>
#include <utils/optionlist.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <regex>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "plutosdr_source",
    /* Description:     */ "PlutoSDR source module for SDR++",
    /* Author:          */ "Ryzerth",
    /* Version:         */ 0, 2, 0,
    /* Max instances    */ 1
};

ConfigManager config;

class PlutoSDRSourceModule;
std::vector<PlutoSDRSourceModule*> instances;   // stopped at exit, see _END_

const std::vector<const char*> deviceWhiteList = {
    "PlutoSDR",
    "ANTSDR",
    "LibreSDR"
};

class PlutoSDRSourceModule : public ModuleManager::Instance {
public:
    PlutoSDRSourceModule(std::string name) {
        this->name = name;

        // Define valid samplerates
        defineSamplerates(false);

        // Define valid bandwidths
        bandwidths.define(0, "Auto", 0);
        for (int bw = 1000000.0; bw <= 52000000; bw += 500000) {
            bandwidths.define(bw, getBandwdithScaled(bw), bw);
        }

        // Define gain modes
        gainModes.define("manual", "Manual", "manual");
        gainModes.define("fast_attack", "Fast Attack", "fast_attack");
        gainModes.define("slow_attack", "Slow Attack", "slow_attack");
        gainModes.define("hybrid", "Hybrid", "hybrid");

        // Define receive ports. RX2 exists on two-receiver boards (2R2T mode);
        // a one-receiver Pluto reports an error if RX2 is selected.
        rxPorts.define(0, "RX1", 0);
        rxPorts.define(1, "RX2", 1);

        // Enumerate devices
        refresh();

        // Select device
        config.acquire();
        devDesc = config.conf["device"];
        config.release();
        select(devDesc);

        // Register source
        handler.ctx = this;
        handler.selectHandler = menuSelected;
        handler.deselectHandler = menuDeselected;
        handler.menuHandler = menuHandler;
        handler.startHandler = start;
        handler.stopHandler = stop;
        handler.tuneHandler = tune;
        handler.stream = &stream;
        sigpath::sourceManager.registerSource("PlutoSDR", &handler);
    }

    ~PlutoSDRSourceModule() {
        stop(this);
        sigpath::sourceManager.unregisterSource("PlutoSDR");
    }

    void postInit() {}

    // Called at exit: stop streaming, which also puts the FPGA decimator back.
    void shutdown() { stop(this); }

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = true;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    // The rates offered depend on the FPGA /8 decimator. Without it, the AD9361
    // delivers the rate directly. With it, the AD9361 runs at 8x the rate shown
    // and the FPGA filters and decimates, so rates below the AD9361's own
    // ~2.08 MS/s minimum are clean and the link carries an eighth of the data.
    void defineSamplerates(bool decim) {
        samplerates.clear();
        if (decim) {
            const int rates[] = { 250000, 300000, 400000, 500000, 600000, 750000, 1000000,
                                  1500000, 2000000, 2500000, 3000000, 4000000, 5000000, 6000000 };
            for (int sr : rates) { samplerates.define(sr, getBandwdithScaled(sr), sr); }
            samplerates.define(7680000, "7.68MHz", 7680000);   // 61.44 MS/s / 8, the most it can give
            return;
        }
        for (int sr = 1000000; sr <= 61440000; sr += 500000) {
            samplerates.define(sr, getBandwdithScaled(sr), sr);
        }
        samplerates.define(61440000, getBandwdithScaled(61440000.0), 61440000.0);
    }

    // Select the offered rate closest to the current one.
    void selectNearestSamplerate() {
        int best = 0;
        for (int i = 0; i < samplerates.size(); i++) {
            if (std::abs(samplerates.value(i) - samplerate) < std::abs(samplerates.value(best) - samplerate)) { best = i; }
        }
        srId = best;
        samplerate = samplerates.value(srId);
    }

    void saveConf(const char* key, const json& v) {
        if (devDesc.empty()) { return; }
        config.acquire();
        config.conf["devices"][devDesc][key] = v;
        config.release(true);
    }

    std::string getBandwdithScaled(double bw) {
        char buf[1024];
        if (bw >= 1000000.0) {
            sprintf(buf, "%.1lfMHz", bw / 1000000.0);
        }
        else if (bw >= 1000.0) {
            sprintf(buf, "%.1lfKHz", bw / 1000.0);
        }
        else {
            sprintf(buf, "%.1lfHz", bw);
        }
        return std::string(buf);
    }

    void refresh() {
        // Clear device list
        devices.clear();

        // Create scan context
        iio_scan_context* sctx = iio_create_scan_context(NULL, 0);
        if (!sctx) {
            flog::error("Failed get scan context");
            return;
        }

        // Create parsing regexes
        std::regex backendRgx(".+(?=:)", std::regex::ECMAScript);
        std::regex modelRgx("\\(.+(?=\\),)", std::regex::ECMAScript);
        std::regex serialRgx("serial=[0-9A-Za-z]+", std::regex::ECMAScript);

        // Enumerate devices
        iio_context_info** ctxInfoList;
        ssize_t count = iio_scan_context_get_info_list(sctx, &ctxInfoList);
        if (count < 0) {
            flog::error("Failed to enumerate contexts");
            return;
        }
        for (ssize_t i = 0; i < count; i++) {
            iio_context_info* info = ctxInfoList[i];
            std::string desc = iio_context_info_get_description(info);
            std::string duri = iio_context_info_get_uri(info);

            // If the device is not a plutosdr, don't include it
            bool isPluto = false;
            for (const auto type : deviceWhiteList) {
                if (desc.find(type) != std::string::npos) {
                    isPluto = true;
                    break;
                }
            }
            if (!isPluto) {
                flog::warn("Ignored IIO device: [{}] {}", duri, desc);
                continue;
            }

            // Extract the backend
            std::string backend = "unknown";
            std::smatch backendMatch;
            if (std::regex_search(duri, backendMatch, backendRgx)) {
                backend = backendMatch[0];
            }

            // Extract the model
            std::string model = "Unknown";
            std::smatch modelMatch;
            if (std::regex_search(desc, modelMatch, modelRgx)) {
                model = modelMatch[0];
                int parenthPos = model.find('(');
                if (parenthPos != std::string::npos) {
                    model = model.substr(parenthPos+1);
                }
            }

            // Extract the serial
            std::string serial = "unknown";
            std::smatch serialMatch;
            if (std::regex_search(desc, serialMatch, serialRgx)) {
                serial = serialMatch[0].str().substr(7);
            }

            // Construct the device name
            std::string devName = '(' + backend + ") " + model + " [" + serial + ']';

            // Skip duplicate devices
            if (devices.keyExists(desc) || devices.nameExists(devName) || devices.valueExists(duri)) { continue; }

            // Save device
            devices.define(desc, devName, duri);
        }
        iio_context_info_list_free(ctxInfoList);
        
        // Destroy scan context
        iio_scan_context_destroy(sctx);

#ifdef __ANDROID__
        // On Android, a default IP entry must be made (TODO: This is not ideal since the IP cannot be changed)
        const char* androidURI = "ip:192.168.2.1";
        const char* androidName = "Default (192.168.2.1)";
        devices.define(androidName, androidName, androidURI);
#endif
    }

    void select(const std::string& desc) {
        // If no device is available, give up
        if (devices.empty()) {
            devDesc.clear();
            return;
        }

        // If the device is not available, select the first one
        if (!devices.keyExists(desc)) {
            select(devices.key(0));
        }

        // Update URI
        devDesc = desc;
        uri = devices.value(devices.keyId(desc));

        // TODO: Enumerate capabilities

        // Load defaults
        samplerate = 4000000;
        bandwidth = 0;
        gmId = 0;
        gain = -1.0f;
        rxPort = 0;
        fpgaDecim = false;
        quadTrack = rfDcTrack = bbDcTrack = true;
        ppm = 0.0f;
        ppmSet = false;

        // Load device config
        config.acquire();
        if (config.conf["devices"][devDesc].contains("samplerate")) {
            samplerate = config.conf["devices"][devDesc]["samplerate"];
        }
        if (config.conf["devices"][devDesc].contains("bandwidth")) {
            bandwidth = config.conf["devices"][devDesc]["bandwidth"];
        }
        if (config.conf["devices"][devDesc].contains("gainMode")) {
            // Select given gain mode or default if invalid
            std::string gm = config.conf["devices"][devDesc]["gainMode"];
            if (gainModes.keyExists(gm)) {
                gmId = gainModes.keyId(gm);
            }
            else {
                gmId = 0;
            }
        }
        if (config.conf["devices"][devDesc].contains("gain")) {
            gain = config.conf["devices"][devDesc]["gain"];
            gain = std::clamp<int>(gain, -1.0f, 73.0f);
        }
        if (config.conf["devices"][devDesc].contains("rxPort")) {
            rxPort = config.conf["devices"][devDesc]["rxPort"];
        }
        json& dc = config.conf["devices"][devDesc];
        if (dc.contains("fpgaDecimation")) { fpgaDecim = dc["fpgaDecimation"]; }
        if (dc.contains("quadratureTracking")) { quadTrack = dc["quadratureTracking"]; }
        if (dc.contains("rfDcTracking")) { rfDcTrack = dc["rfDcTracking"]; }
        if (dc.contains("bbDcTracking")) { bbDcTrack = dc["bbDcTracking"]; }
        if (dc.contains("ppm")) { ppm = dc["ppm"]; ppmSet = true; }
        config.release();
        defineSamplerates(fpgaDecim);

        // Update RX port ID
        if (rxPorts.keyExists(rxPort)) {
            rxPortId = rxPorts.keyId(rxPort);
        }
        else {
            rxPortId = 0;
            rxPort = rxPorts.value(rxPortId);
        }

        // Update samplerate ID
        if (samplerates.keyExists(samplerate)) {
            srId = samplerates.keyId(samplerate);
        }
        else {
            srId = 0;
            samplerate = samplerates.value(srId);
        }

        // Update bandwidth ID
        if (bandwidths.keyExists(bandwidth)) {
            bwId = bandwidths.keyId(bandwidth);
        }
        else {
            bwId = 0;
            bandwidth = bandwidths.value(bwId);
        }
    }

    static void menuSelected(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        core::setInputSampleRate(_this->samplerate);
        flog::info("PlutoSDRSourceModule '{0}': Menu Select!", _this->name);
    }

    static void menuDeselected(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        flog::info("PlutoSDRSourceModule '{0}': Menu Deselect!", _this->name);
    }

    static void start(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        if (_this->running) { return; }

        // If no device is selected, give up
        if (_this->devDesc.empty() || _this->uri.empty()) { return; }

        // Open context
        _this->ctx = iio_create_context_from_uri(_this->uri.c_str());
        if (_this->ctx == NULL) {
            flog::error("Could not open pluto ({})", _this->uri);
            return;
        }

        // Get phy and device handle
        _this->phy = iio_context_find_device(_this->ctx, "ad9361-phy");
        if (_this->phy == NULL) {
            flog::error("Could not connect to pluto phy");
            iio_context_destroy(_this->ctx);
            return;
        }
        _this->dev = iio_context_find_device(_this->ctx, "cf-ad9361-lpc");
        if (_this->dev == NULL) {
            flog::error("Could not connect to pluto dev");
            iio_context_destroy(_this->ctx);
            return;
        }

        // Get RX channels. Sample rate, filter, bandwidth and port select are
        // shared by both receivers and set through RX1's channel; gain and gain
        // mode belong to the selected receiver.
        _this->rxChan0 = iio_device_find_channel(_this->phy, "voltage0", false);
        _this->rxChan = iio_device_find_channel(_this->phy, _this->rxPort ? "voltage1" : "voltage0", false);
        _this->rxLO = iio_device_find_channel(_this->phy, "altvoltage0", true);
        if (!_this->rxChan0 || !_this->rxChan) {
            flog::error("This device has no RX{} (it may be a one-receiver Pluto)", _this->rxPort + 1);
            iio_context_destroy(_this->ctx);
            _this->ctx = NULL;
            return;
        }

        // Enable RX LO and disable TX
        iio_channel_attr_write_bool(iio_device_find_channel(_this->phy, "altvoltage1", true), "powerdown", true);
        iio_channel_attr_write_bool(_this->rxLO, "powerdown", false);

        // The AD9361 runs at 8x the delivered rate when the FPGA decimates.
        long long adRate = round(_this->samplerate) * (_this->fpgaDecim ? 8 : 1);

        // Configure RX channel
        iio_channel_attr_write(_this->rxChan0, "rf_port_select", "A_BALANCED");
        iio_channel_attr_write_longlong(_this->rxLO, "frequency", round(_this->freq));                              // Freq
        iio_channel_attr_write_bool(_this->rxChan0, "filter_fir_en", true);                                         // Digital filter
        iio_channel_attr_write_longlong(_this->rxChan0, "sampling_frequency", adRate);                              // Sample rate
        // Gain mode before gain: in an automatic mode the chip refuses a gain
        // write, and switching to manual afterwards keeps the automatic value.
        iio_channel_attr_write(_this->rxChan, "gain_control_mode", _this->gainModes.value(_this->gmId).c_str());    // Gain mode
        iio_channel_attr_write_double(_this->rxChan, "hardwaregain", _this->gain);                                  // Gain
        _this->setBandwidth(_this->bandwidth);

        // Configure the ADC filters
        ad9361_set_bb_rate(_this->phy, adRate);

        // The FPGA decimator is chosen by the capture device's own rate: the AD
        // rate bypasses it, an eighth of it engages it. Read it back: a design
        // without the decimator refuses, and the stream would then run 8x fast.
        _this->lpcChan = iio_device_find_channel(_this->dev, "voltage0", false);
        long long want = _this->fpgaDecim ? adRate / 8 : adRate;
        long long got = 0;
        if (_this->lpcChan) {
            iio_channel_attr_write_longlong(_this->lpcChan, "sampling_frequency", want);
            iio_channel_attr_read_longlong(_this->lpcChan, "sampling_frequency", &got);
        }
        if (_this->fpgaDecim && got != want) {
            flog::error("This device's FPGA did not engage the /8 decimator (asked {}, got {}); turn it off", std::to_string(want), std::to_string(got));
            if (_this->lpcChan) { iio_channel_attr_write_longlong(_this->lpcChan, "sampling_frequency", adRate); }
            _this->decimFailed = true;
            iio_context_destroy(_this->ctx);
            _this->ctx = NULL;
            return;
        }
        _this->decimFailed = false;

        // DC and IQ correction, shared by both receivers
        _this->writeTracking();

        // Frequency correction: apply the saved value, or show the board's own
        if (_this->ppmSet) { _this->writePpm(); }
        else { _this->readPpm(); }

        // What this board is, for the status line
        const char* v = iio_context_get_attr_value(_this->ctx, "hw_model");
        _this->statusModel = v ? v : "unknown";
        v = iio_context_get_attr_value(_this->ctx, "fw_build");
        if (!v) { v = iio_context_get_attr_value(_this->ctx, "fw_version"); }
        _this->statusFw = v ? v : "unknown";
        _this->xadc = iio_context_find_device(_this->ctx, "xadc");
        _this->statusTemps.clear();
        _this->lastStatus = std::chrono::steady_clock::time_point();

        // Start worker thread
        _this->running = true;
        _this->workerThread = std::thread(worker, _this);
        flog::info("PlutoSDRSourceModule '{0}': Start on RX{1}!", _this->name, _this->rxPort + 1);
    }

    static void stop(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        if (!_this->running) { return; }

        // Stop worker thread
        _this->running = false;
        _this->stream.stopWriter();
        _this->workerThread.join();
        _this->stream.clearWriteStop();

        // Leave the FPGA decimator bypassed, as other programs expect
        if (_this->ctx != NULL && _this->fpgaDecim && _this->lpcChan) {
            long long adRate = 0;
            if (iio_channel_attr_read_longlong(_this->rxChan0, "sampling_frequency", &adRate) == 0 && adRate > 0) {
                iio_channel_attr_write_longlong(_this->lpcChan, "sampling_frequency", adRate);
            }
        }

        // Close device
        if (_this->ctx != NULL) {
            iio_context_destroy(_this->ctx);
            _this->ctx = NULL;
        }

        flog::info("PlutoSDRSourceModule '{0}': Stop!", _this->name);
    }

    static void tune(double freq, void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        _this->freq = freq;
        if (_this->running) {
            // Tune device
            iio_channel_attr_write_longlong(_this->rxLO, "frequency", round(freq));
        }
        flog::info("PlutoSDRSourceModule '{0}': Tune: {1}!", _this->name, freq);
    }

    static void menuHandler(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;

        if (_this->running) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo("##plutosdr_dev_sel", &_this->devId, _this->devices.txt)) {
            _this->select(_this->devices.key(_this->devId));
            core::setInputSampleRate(_this->samplerate);
            config.acquire();
            config.conf["device"] = _this->devices.key(_this->devId);
            config.release(true);
        }

        if (SmGui::Combo(CONCAT("##_pluto_sr_", _this->name), &_this->srId, _this->samplerates.txt)) {
            _this->samplerate = _this->samplerates.value(_this->srId);
            core::setInputSampleRate(_this->samplerate);
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["samplerate"] = _this->samplerate;
                config.release(true);
            }
        }

        // Refresh button
        SmGui::SameLine();
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Button(CONCAT("Refresh##_pluto_refr_", _this->name))) {
            _this->refresh();
            _this->select(_this->devDesc);
            core::setInputSampleRate(_this->samplerate);
        }

        SmGui::LeftLabel("RX Port");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_pluto_rxport_", _this->name), &_this->rxPortId, _this->rxPorts.txt)) {
            _this->rxPort = _this->rxPorts.value(_this->rxPortId);
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["rxPort"] = _this->rxPort;
                config.release(true);
            }
        }

        // The FPGA /8 decimator: lower rates, filtered in the FPGA
        if (SmGui::Checkbox(CONCAT("FPGA /8 decimator##_pluto_decim_", _this->name), &_this->fpgaDecim)) {
            _this->defineSamplerates(_this->fpgaDecim);
            _this->selectNearestSamplerate();
            core::setInputSampleRate(_this->samplerate);
            _this->saveConf("fpgaDecimation", _this->fpgaDecim);
            _this->saveConf("samplerate", _this->samplerate);
        }
        if (_this->running) { SmGui::EndDisabled(); }
        if (_this->decimFailed) {
            SmGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "This FPGA has no /8 decimator");
        }

        SmGui::LeftLabel("Bandwidth");
        SmGui::FillWidth();
        if (SmGui::Combo(CONCAT("##_pluto_bw_", _this->name), &_this->bwId, _this->bandwidths.txt)) {
            _this->bandwidth = _this->bandwidths.value(_this->bwId);
            if (_this->running) {
                _this->setBandwidth(_this->bandwidth);
            }
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["bandwidth"] = _this->bandwidth;
                config.release(true);
            }
        }

        SmGui::LeftLabel("Gain Mode");
        SmGui::FillWidth();
        SmGui::ForceSync();
        if (SmGui::Combo(CONCAT("##_pluto_gainmode_select_", _this->name), &_this->gmId, _this->gainModes.txt)) {
            if (_this->running) {
                iio_channel_attr_write(_this->rxChan, "gain_control_mode", _this->gainModes.value(_this->gmId).c_str());
            }
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["gainMode"] = _this->gainModes.key(_this->gmId);
                config.release(true);
            }
        }

        SmGui::LeftLabel("Gain");
        if (_this->gmId) { SmGui::BeginDisabled(); }
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_pluto_gain__", _this->name), &_this->gain, -1.0f, 73.0f, 1.0f, SmGui::FMT_STR_FLOAT_DB_NO_DECIMAL)) {
            if (_this->running) {
                iio_channel_attr_write_double(_this->rxChan, "hardwaregain", _this->gain);
            }
            if (!_this->devDesc.empty()) {
                config.acquire();
                config.conf["devices"][_this->devDesc]["gain"] = _this->gain;
                config.release(true);
            }
        }
        if (_this->gmId) { SmGui::EndDisabled(); }

        // DC and IQ correction: the AD9361's own tracking loops. Named after the
        // chip's attributes, not "IQ correction": SDR++ has its own software
        // "IQ Correction" (a DC blocker on the PC) further down the same menu.
        if (SmGui::Checkbox(CONCAT("Quadrature tracking##_pluto_quad_", _this->name), &_this->quadTrack)) {
            if (_this->running) { _this->writeTracking(); }
            _this->saveConf("quadratureTracking", _this->quadTrack);
        }
        if (SmGui::Checkbox(CONCAT("RF DC tracking##_pluto_rfdc_", _this->name), &_this->rfDcTrack)) {
            if (_this->running) { _this->writeTracking(); }
            _this->saveConf("rfDcTracking", _this->rfDcTrack);
        }
        if (SmGui::Checkbox(CONCAT("Baseband DC tracking##_pluto_bbdc_", _this->name), &_this->bbDcTrack)) {
            if (_this->running) { _this->writeTracking(); }
            _this->saveConf("bbDcTracking", _this->bbDcTrack);
        }

        // Frequency correction of the 40 MHz reference, in ppm
        SmGui::LeftLabel("Freq. corr. (ppm)");
        SmGui::FillWidth();
        if (SmGui::SliderFloatWithSteps(CONCAT("##_pluto_ppm_", _this->name), &_this->ppm, -100.0f, 100.0f, 0.05f, SmGui::FMT_STR_FLOAT_TWO_DECIMAL)) {
            _this->ppmSet = true;
            if (_this->running) { _this->writePpm(); }
            _this->saveConf("ppm", _this->ppm);
        }

        // What the board is, and how warm, refreshed once a second
        if (_this->running) {
            _this->refreshStatus();
            SmGui::Text(("Board: " + _this->statusModel).c_str());
            SmGui::Text(("Firmware: " + _this->statusFw).c_str());
            if (!_this->statusTemps.empty()) { SmGui::Text(_this->statusTemps.c_str()); }
        }
    }

    void writeTracking() {
        iio_channel_attr_write_bool(rxChan0, "quadrature_tracking_en", quadTrack);
        iio_channel_attr_write_bool(rxChan0, "rf_dc_offset_tracking_en", rfDcTrack);
        iio_channel_attr_write_bool(rxChan0, "bb_dc_offset_tracking_en", bbDcTrack);
    }

    // xo_correction is the reference's true frequency in Hz; nominal 40 MHz.
    void writePpm() {
        long long hz = llround(40000000.0 * (1.0 + ppm * 1e-6));
        if (iio_device_attr_write_longlong(phy, "xo_correction", hz) < 0) {
            flog::error("Could not set xo_correction to {} Hz", std::to_string(hz));
        }
    }

    void readPpm() {
        long long hz = 0;
        if (iio_device_attr_read_longlong(phy, "xo_correction", &hz) == 0 && hz > 0) {
            ppm = (float)((hz / 40000000.0 - 1.0) * 1e6);
        }
    }

    void refreshStatus() {
        auto now = std::chrono::steady_clock::now();
        if (now - lastStatus < std::chrono::seconds(1)) { return; }
        lastStatus = now;
        char buf[128];
        double ad = NAN, zynq = NAN;
        long long mdeg;
        iio_channel* t = iio_device_find_channel(phy, "temp0", false);
        if (t && iio_channel_attr_read_longlong(t, "input", &mdeg) == 0) { ad = mdeg / 1000.0; }
        if (xadc) {
            iio_channel* x = iio_device_find_channel(xadc, "temp0", false);
            double raw, off, scale;
            if (x && iio_channel_attr_read_double(x, "raw", &raw) == 0 && iio_channel_attr_read_double(x, "offset", &off) == 0
                && iio_channel_attr_read_double(x, "scale", &scale) == 0) {
                zynq = (raw + off) * scale / 1000.0;
            }
        }
        if (std::isnan(ad) && std::isnan(zynq)) { statusTemps.clear(); return; }
        snprintf(buf, sizeof(buf), "Temp: AD9361 %.1f C, Zynq %.1f C", ad, zynq);
        statusTemps = buf;
    }

    void setBandwidth(int bw) {
        if (bw > 0) {
            iio_channel_attr_write_longlong(rxChan0, "rf_bandwidth", bw);
        }
        else {
            iio_channel_attr_write_longlong(rxChan0, "rf_bandwidth", std::min<int>(samplerate, 52000000));
        }
    }

    static void worker(void* ctx) {
        PlutoSDRSourceModule* _this = (PlutoSDRSourceModule*)ctx;
        // At low rates a 1/200 s block is small (2500 samples at 500 kS/s), and
        // the board overwrites any block not fetched in time: with the default
        // queue, a host a few milliseconds late lost 31% of the samples at
        // 500 kS/s, heard as stutter. So more blocks are queued on the board.
        int blockSize = _this->samplerate / 200.0f;

        // Acquire channels: RX1's I/Q are voltage0/voltage1 of the capture
        // device, RX2's are voltage2/voltage3.
        iio_channel* rx0_i = iio_device_find_channel(_this->dev, _this->rxPort ? "voltage2" : "voltage0", 0);
        iio_channel* rx0_q = iio_device_find_channel(_this->dev, _this->rxPort ? "voltage3" : "voltage1", 0);
        if (!rx0_i || !rx0_q) {
            flog::error("Failed to acquire RX{} channels", _this->rxPort + 1);
            return;
        }

        // Start streaming
        iio_channel_enable(rx0_i);
        iio_channel_enable(rx0_q);

        // Allocate buffer
        if (iio_device_set_kernel_buffers_count(_this->dev, 8) < 0) {
            flog::warn("Could not queue 8 kernel buffers; using the default");
        }
        iio_buffer* rxbuf = iio_device_create_buffer(_this->dev, blockSize, false);
        if (!rxbuf) {
            flog::error("Could not create RX buffer");
            return;
        }

        // Receive loop
        while (true) {
            // Read samples
            iio_buffer_refill(rxbuf);

            // Get buffer pointer
            int16_t* buf = (int16_t*)iio_buffer_first(rxbuf, rx0_i);
            if (!buf) { break; }

            // Convert samples to CF32
            volk_16i_s32f_convert_32f((float*)_this->stream.writeBuf, buf, 32768.0f, blockSize * 2);

            // Send out the samples
            if (!_this->stream.swap(blockSize)) { break; };
        }

        // Stop streaming
        iio_channel_disable(rx0_i);
        iio_channel_disable(rx0_q);

        // Free buffer
        iio_buffer_destroy(rxbuf);
    }

    std::string name;
    bool enabled = true;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    std::thread workerThread;
    iio_context* ctx = NULL;
    iio_device* phy = NULL;
    iio_device* dev = NULL;
    iio_channel* rxLO = NULL;
    iio_channel* rxChan = NULL;   // the selected receiver: gain and gain mode
    iio_channel* rxChan0 = NULL;  // RX1: attributes both receivers share
    iio_channel* lpcChan = NULL;  // the capture device's rate: picks the FPGA decimation
    iio_device* xadc = NULL;      // the Zynq's own temperature sensor
    bool running = false;

    bool fpgaDecim = false;
    bool decimFailed = false;
    bool quadTrack = true;
    bool rfDcTrack = true;
    bool bbDcTrack = true;
    float ppm = 0.0f;
    bool ppmSet = false;
    std::string statusModel;
    std::string statusFw;
    std::string statusTemps;
    std::chrono::steady_clock::time_point lastStatus;

    std::string devDesc = "";
    std::string uri = "";

    double freq;
    int samplerate = 4000000;
    int bandwidth = 0;
    float gain = -1;

    int devId = 0;
    int srId = 0;
    int bwId = 0;
    int gmId = 0;
    int rxPort = 0;
    int rxPortId = 0;

    OptionList<std::string, std::string> devices;
    OptionList<int, double> samplerates;
    OptionList<int, double> bandwidths;
    OptionList<std::string, std::string> gainModes;
    OptionList<int, int> rxPorts;
};

MOD_EXPORT void _INIT_() {
    json defConf = {};
    defConf["device"] = "";
    defConf["devices"] = {};
    config.setPath(core::args["root"].s() + "/plutosdr_source_config.json");
    config.load(defConf);
    config.enableAutoSave();

    // Reset the configuration if the old format is still used
    config.acquire();
    if (!config.conf.contains("device") || !config.conf.contains("devices")) {
        config.conf = defConf;
        config.release(true);
    }
    else {
        config.release();
    }
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    PlutoSDRSourceModule* inst = new PlutoSDRSourceModule(name);
    instances.push_back(inst);
    return inst;
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    instances.erase(std::remove(instances.begin(), instances.end(), (PlutoSDRSourceModule*)instance), instances.end());
    delete (PlutoSDRSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    // SDR++ ends modules on exit without stopping their sources, so a source
    // left playing would leave the FPGA decimator engaged for the next program.
    for (auto inst : instances) { inst->shutdown(); }
    config.disableAutoSave();
    config.save();
}