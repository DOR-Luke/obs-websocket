/*
obs-websocket
Copyright (C) 2014 by Leonhard Oelke <leonhard@in-verted.de>
Copyright (C) 2016-2021 Stephane Lepin <stephane.lepin@gmail.com>
Copyright (C) 2020-2021 Kyle Manning <tt2468@gmail.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>
*/

#include <cmath>
#include <algorithm>

#include "Obs.h"
#include "Obs_VolumeMeter.h"
#include "Obs_VolumeMeter_Helpers.h"
#include "../obs-websocket.h"

Utils::Obs::VolumeMeter::Meter::Meter(obs_source_t *input)
	: PeakMeterType(SAMPLE_PEAK_METER),
	  _input(obs_source_get_weak_source(input)),
	  _channels(0),
	  _lastUpdate(0),
	  _volume(obs_source_get_volume(input))
{
	signal_handler_t *sh = obs_source_get_signal_handler(input);
	signal_handler_connect(sh, "volume", Meter::InputVolumeCallback, this);

	obs_source_add_audio_capture_callback(input, Meter::InputAudioCaptureCallback, this);

	blog_debug("[Utils::Obs::VolumeMeter::Meter::Meter] Meter created for input: %s", obs_source_get_name(input));
}

Utils::Obs::VolumeMeter::Meter::~Meter()
{
	OBSSourceAutoRelease input = obs_weak_source_get_source(_input);
	if (!input) {
		blog(LOG_WARNING,
		     "[Utils::Obs::VolumeMeter::Meter::~Meter] Failed to get strong reference to input. Has it been destroyed?");
		return;
	}

	signal_handler_t *sh = obs_source_get_signal_handler(input);
	signal_handler_disconnect(sh, "volume", Meter::InputVolumeCallback, this);

	obs_source_remove_audio_capture_callback(input, Meter::InputAudioCaptureCallback, this);

	blog_debug("[Utils::Obs::VolumeMeter::Meter::~Meter] Meter destroyed for input: %s", obs_source_get_name(input));
}

bool Utils::Obs::VolumeMeter::Meter::InputValid()
{
	return !obs_weak_source_expired(_input);
}

json Utils::Obs::VolumeMeter::Meter::GetMeterData()
{
	json ret;

	OBSSourceAutoRelease input = obs_weak_source_get_source(_input);
	if (!input) {
		blog(LOG_WARNING,
		     "[Utils::Obs::VolumeMeter::Meter::GetMeterData] Failed to get strong reference to input. Has it been destroyed?");
		return ret;
	}

	std::vector<std::vector<float>> levels;
	const float volume = _muted ? 0.0f : _volume.load();

	std::unique_lock<std::mutex> l(_mutex);

	if (_lastUpdate != 0 && (os_gettime_ns() - _lastUpdate) * 0.000000001 > 0.3)
		ResetAudioLevels();

	for (int channel = 0; channel < _channels; channel++) {
		std::vector<float> level;
		level.push_back(_magnitude[channel] * volume);
		level.push_back(_peak[channel] * volume);
		level.push_back(_peak[channel]);

		levels.push_back(level);
	}
	l.unlock();

	ret["inputName"] = obs_source_get_name(input);
	ret["inputUuid"] = obs_source_get_uuid(input);
	ret["inputLevelsMul"] = levels;

	return ret;
}

// MUST HOLD LOCK
void Utils::Obs::VolumeMeter::Meter::ResetAudioLevels()
{
	_lastUpdate = 0;
	for (int channelNumber = 0; channelNumber < MAX_AUDIO_CHANNELS; channelNumber++) {
		_magnitude[channelNumber] = 0;
		_peak[channelNumber] = 0;
	}
}

// MUST HOLD LOCK
void Utils::Obs::VolumeMeter::Meter::ProcessAudioChannels(const struct audio_data *data)
{
	int channels = 0;
	for (int i = 0; i < MAX_AV_PLANES; i++) {
		if (data->data[i])
			channels++;
	}

	bool channelsChanged = _channels != channels;
	_channels = std::clamp(channels, 0, MAX_AUDIO_CHANNELS);

	if (channelsChanged)
		ResetAudioLevels();
}

// MUST HOLD LOCK
void Utils::Obs::VolumeMeter::Meter::ProcessPeak(const struct audio_data *data)
{
	size_t sampleCount = data->frames;
	int channelNumber = 0;

	for (int planeNumber = 0; channelNumber < _channels; planeNumber++) {
		float *samples = (float *)data->data[planeNumber];
		if (!samples)
			continue;

		if (((uintptr_t)samples & 0xf) > 0) {
			_peak[channelNumber] = 1.0f;
			channelNumber++;
			continue;
		}

		__m128 previousSamples = _mm_loadu_ps(_previousSamples[channelNumber]);

		float peak;
		switch (PeakMeterType) {
		default:
		case SAMPLE_PEAK_METER:
			peak = GetSamplePeak(previousSamples, samples, sampleCount);
			break;
		case TRUE_PEAK_METER:
			peak = GetTruePeak(previousSamples, samples, sampleCount);
			break;
		}

		switch (sampleCount) {
		case 0:
			break;
		case 1:
			_previousSamples[channelNumber][0] = _previousSamples[channelNumber][1];
			_previousSamples[channelNumber][1] = _previousSamples[channelNumber][2];
			_previousSamples[channelNumber][2] = _previousSamples[channelNumber][3];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
			break;
		case 2:
			_previousSamples[channelNumber][0] = _previousSamples[channelNumber][2];
			_previousSamples[channelNumber][1] = _previousSamples[channelNumber][3];
			_previousSamples[channelNumber][2] = samples[sampleCount - 2];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
			break;
		case 3:
			_previousSamples[channelNumber][0] = _previousSamples[channelNumber][3];
			_previousSamples[channelNumber][1] = samples[sampleCount - 3];
			_previousSamples[channelNumber][2] = samples[sampleCount - 2];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
			break;
		default:
			_previousSamples[channelNumber][0] = samples[sampleCount - 4];
			_previousSamples[channelNumber][1] = samples[sampleCount - 3];
			_previousSamples[channelNumber][2] = samples[sampleCount - 2];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
		}

		_peak[channelNumber] = peak;

		channelNumber++;
	}

	for (; channelNumber < MAX_AUDIO_CHANNELS; channelNumber++)
		_peak[channelNumber] = 0.0;
}

// MUST HOLD LOCK
void Utils::Obs::VolumeMeter::Meter::ProcessMagnitude(const struct audio_data *data)
{
	size_t sampleCount = data->frames;

	int channelNumber = 0;
	for (int planeNumber = 0; channelNumber < _channels; planeNumber++) {
		float *samples = (float *)data->data[planeNumber];
		if (!samples)
			continue;

		float sum = 0.0;
		for (size_t i = 0; i < sampleCount; i++) {
			float sample = samples[i];
			sum += sample * sample;
		}

		_magnitude[channelNumber] = std::sqrt(sum / sampleCount);

		channelNumber++;
	}
}

void Utils::Obs::VolumeMeter::Meter::InputAudioCaptureCallback(void *priv_data, obs_source_t *, const struct audio_data *data,
							       bool muted)
{
	auto c = static_cast<Meter *>(priv_data);

	std::unique_lock<std::mutex> l(c->_mutex);

	c->_muted = muted;
	c->ProcessAudioChannels(data);
	c->ProcessPeak(data);
	c->ProcessMagnitude(data);

	c->_lastUpdate = os_gettime_ns();
}

void Utils::Obs::VolumeMeter::Meter::InputVolumeCallback(void *priv_data, calldata_t *cd)
{
	auto c = static_cast<Meter *>(priv_data);

	c->_volume = (float)calldata_float(cd, "volume");
}

// OutputMeter Implementation
Utils::Obs::VolumeMeter::OutputMeter::OutputMeter(obs_source_t *output)
	: PeakMeterType(SAMPLE_PEAK_METER),
	  _output(obs_source_get_weak_source(output)),
	  _channels(0),
	  _lastUpdate(0),
	  _volume(obs_source_get_volume(output))
{
	signal_handler_t *sh = obs_source_get_signal_handler(output);
	signal_handler_connect(sh, "volume", OutputMeter::OutputVolumeCallback, this);

	obs_source_add_audio_capture_callback(output, OutputMeter::OutputAudioCaptureCallback, this);

	blog_debug("[Utils::Obs::VolumeMeter::OutputMeter::OutputMeter] OutputMeter created for output: %s (ID: %s)", 
	          obs_source_get_name(output), obs_source_get_id(output));
}

Utils::Obs::VolumeMeter::OutputMeter::~OutputMeter()
{
	OBSSourceAutoRelease output = obs_weak_source_get_source(_output);
	if (!output) {
		blog(LOG_WARNING,
		     "[Utils::Obs::VolumeMeter::OutputMeter::~OutputMeter] Failed to get strong reference to output. Has it been destroyed?");
		return;
	}

	signal_handler_t *sh = obs_source_get_signal_handler(output);
	signal_handler_disconnect(sh, "volume", OutputMeter::OutputVolumeCallback, this);

	obs_source_remove_audio_capture_callback(output, OutputMeter::OutputAudioCaptureCallback, this);

	blog_debug("[Utils::Obs::VolumeMeter::OutputMeter::~OutputMeter] OutputMeter destroyed for output: %s", obs_source_get_name(output));
}

bool Utils::Obs::VolumeMeter::OutputMeter::OutputValid()
{
	return !obs_weak_source_expired(_output);
}

json Utils::Obs::VolumeMeter::OutputMeter::GetMeterData()
{
	json ret;

	OBSSourceAutoRelease output = obs_weak_source_get_source(_output);
	if (!output) {
		blog(LOG_WARNING,
		     "[Utils::Obs::VolumeMeter::OutputMeter::GetMeterData] Failed to get strong reference to output. Has it been destroyed?");
		return ret;
	}

	std::vector<std::vector<float>> levels;
	const float volume = _muted ? 0.0f : _volume.load();

	std::unique_lock<std::mutex> l(_mutex);

	if (_lastUpdate != 0 && (os_gettime_ns() - _lastUpdate) * 0.000000001 > 0.3)
		ResetAudioLevels();

	for (int channel = 0; channel < _channels; channel++) {
		std::vector<float> level;
		level.push_back(_magnitude[channel] * volume);
		level.push_back(_peak[channel] * volume);
		level.push_back(_peak[channel]);

		levels.push_back(level);
	}
	l.unlock();

	ret["outputName"] = obs_source_get_name(output);
	ret["outputUuid"] = obs_source_get_uuid(output);
	ret["outputLevelsMul"] = levels;

	return ret;
}

// MUST HOLD LOCK
void Utils::Obs::VolumeMeter::OutputMeter::ResetAudioLevels()
{
	_lastUpdate = 0;
	for (int channelNumber = 0; channelNumber < MAX_AUDIO_CHANNELS; channelNumber++) {
		_magnitude[channelNumber] = 0;
		_peak[channelNumber] = 0;
	}
}

void Utils::Obs::VolumeMeter::OutputMeter::ProcessAudioChannels(const struct audio_data *data)
{
	int channels = 0;
	for (int i = 0; i < MAX_AV_PLANES; i++) {
		if (data->data[i])
			channels++;
	}

	bool channelsChanged = _channels != channels;
	_channels = std::clamp(channels, 0, MAX_AUDIO_CHANNELS);

	if (channelsChanged)
		ResetAudioLevels();
}

void Utils::Obs::VolumeMeter::OutputMeter::ProcessPeak(const struct audio_data *data)
{
	size_t sampleCount = data->frames;
	int channelNumber = 0;

	for (int planeNumber = 0; channelNumber < _channels; planeNumber++) {
		float *samples = (float *)data->data[planeNumber];
		if (!samples)
			continue;

		if (((uintptr_t)samples & 0xf) > 0) {
			_peak[channelNumber] = 1.0f;
			channelNumber++;
			continue;
		}

		__m128 previousSamples = _mm_loadu_ps(_previousSamples[channelNumber]);

		float peak;
		switch (PeakMeterType) {
		default:
		case SAMPLE_PEAK_METER:
			peak = GetSamplePeak(previousSamples, samples, sampleCount);
			break;
		case TRUE_PEAK_METER:
			peak = GetTruePeak(previousSamples, samples, sampleCount);
			break;
		}

		switch (sampleCount) {
		case 0:
			break;
		case 1:
			_previousSamples[channelNumber][0] = _previousSamples[channelNumber][1];
			_previousSamples[channelNumber][1] = _previousSamples[channelNumber][2];
			_previousSamples[channelNumber][2] = _previousSamples[channelNumber][3];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
			break;
		case 2:
			_previousSamples[channelNumber][0] = _previousSamples[channelNumber][2];
			_previousSamples[channelNumber][1] = _previousSamples[channelNumber][3];
			_previousSamples[channelNumber][2] = samples[sampleCount - 2];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
			break;
		case 3:
			_previousSamples[channelNumber][0] = _previousSamples[channelNumber][3];
			_previousSamples[channelNumber][1] = samples[sampleCount - 3];
			_previousSamples[channelNumber][2] = samples[sampleCount - 2];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
			break;
		default:
			_previousSamples[channelNumber][0] = samples[sampleCount - 4];
			_previousSamples[channelNumber][1] = samples[sampleCount - 3];
			_previousSamples[channelNumber][2] = samples[sampleCount - 2];
			_previousSamples[channelNumber][3] = samples[sampleCount - 1];
		}

		_peak[channelNumber] = peak;

		channelNumber++;
	}

	for (; channelNumber < MAX_AUDIO_CHANNELS; channelNumber++)
		_peak[channelNumber] = 0.0;
}

// MUST HOLD LOCK
void Utils::Obs::VolumeMeter::OutputMeter::ProcessMagnitude(const struct audio_data *data)
{
	size_t sampleCount = data->frames;

	int channelNumber = 0;
	for (int planeNumber = 0; channelNumber < _channels; planeNumber++) {
		float *samples = (float *)data->data[planeNumber];
		if (!samples)
			continue;

		float sum = 0.0;
		for (size_t i = 0; i < sampleCount; i++) {
			float sample = samples[i];
			sum += sample * sample;
		}

		_magnitude[channelNumber] = std::sqrt(sum / sampleCount);

		channelNumber++;
	}
}

void Utils::Obs::VolumeMeter::OutputMeter::OutputAudioCaptureCallback(void *priv_data, obs_source_t *, const struct audio_data *data,
								       bool muted)
{
	auto c = static_cast<OutputMeter *>(priv_data);

	std::unique_lock<std::mutex> l(c->_mutex);

	c->_muted = muted;
	c->ProcessAudioChannels(data);
	c->ProcessPeak(data);
	c->ProcessMagnitude(data);

	c->_lastUpdate = os_gettime_ns();
}

void Utils::Obs::VolumeMeter::OutputMeter::OutputVolumeCallback(void *priv_data, calldata_t *cd)
{
	auto c = static_cast<OutputMeter *>(priv_data);

	c->_volume = (float)calldata_float(cd, "volume");
}

Utils::Obs::VolumeMeter::Handler::Handler(UpdateCallback cb, uint64_t updatePeriod)
	: _updateCallback(cb),
	  _updatePeriod(updatePeriod),
	  _running(false)
{
	blog_debug("[Utils::Obs::VolumeMeter::Handler::Handler] Constructor called!");
	
	signal_handler_t *sh = obs_get_signal_handler();
	if (!sh) {
		blog_debug("[Utils::Obs::VolumeMeter::Handler::Handler] No signal handler found!");
		return;
	}

	auto enumProc = [](void *priv_data, obs_source_t *source) {
		auto c = static_cast<Handler *>(priv_data);

		const char *sourceName = obs_source_get_name(source);
		const char *sourceId = obs_source_get_id(source);
		obs_source_type sourceType = obs_source_get_type(source);
		bool isActive = obs_source_active(source);
		uint32_t flags = obs_source_get_output_flags(source);
		bool hasAudio = (flags & OBS_SOURCE_AUDIO) != 0;

		// 모든 소스 정보를 로그로 출력
		blog_debug("[Handler::enumProc] Source: '%s', ID: '%s', Type: %d, Active: %s, HasAudio: %s", 
		          sourceName ? sourceName : "NULL", 
		          sourceId ? sourceId : "NULL", 
		          sourceType, 
		          isActive ? "true" : "false",
		          hasAudio ? "true" : "false");

		if (!isActive)
			return true;

		if (!hasAudio)
			return true;

		// Add input sources
		if (sourceType == OBS_SOURCE_TYPE_INPUT) {
			c->_meters.emplace_back(std::move(new Meter(source)));
			blog_debug("[Handler] Added INPUT meter for: %s (ID: %s)", sourceName, sourceId);
		}

		// Add output sources - 더 넓은 범위로 검사
		if (sourceId) {
			bool isOutputSource = false;
			
			// Windows
			if (strstr(sourceId, "wasapi_output_capture") || 
			    strstr(sourceId, "wasapi_process_output_capture")) {
				isOutputSource = true;
			}
			// Linux
			else if (strstr(sourceId, "pulse_output_capture") ||
			         strstr(sourceId, "alsa_output_capture")) {
				isOutputSource = true;
			}
			// macOS
			else if (strstr(sourceId, "coreaudio_output_capture")) {
				isOutputSource = true;
			}
			// 일반적인 데스크탑 오디오 이름 패턴도 확인
			else if (sourceName && (strstr(sourceName, "Desktop Audio") || 
			                       strstr(sourceName, "데스크탑 오디오") ||
			                       strstr(sourceName, "System Audio") ||
			                       strstr(sourceName, "Speaker") ||
			                       strstr(sourceName, "스피커"))) {
				isOutputSource = true;
				blog_debug("[Handler] Found output source by NAME pattern: %s", sourceName);
			}

			if (isOutputSource) {
				c->_outputMeters.emplace_back(std::move(new OutputMeter(source)));
				blog_debug("[Handler] *** ADDED OUTPUT METER *** for: '%s' (ID: '%s')", sourceName, sourceId);
			}
		}

		return true;
	};
	obs_enum_sources(enumProc, this);

	signal_handler_connect(sh, "source_activate", Handler::InputActivateCallback, this);
	signal_handler_connect(sh, "source_deactivate", Handler::InputDeactivateCallback, this);

	_running = true;
	_updateThread = std::thread(&Handler::UpdateThread, this);

	blog_debug("[Utils::Obs::VolumeMeter::Handler::Handler] Handler created.");
}

Utils::Obs::VolumeMeter::Handler::~Handler()
{
	signal_handler_t *sh = obs_get_signal_handler();
	if (!sh)
		return;

	signal_handler_disconnect(sh, "source_activate", Handler::InputActivateCallback, this);
	signal_handler_disconnect(sh, "source_deactivate", Handler::InputDeactivateCallback, this);

	if (_running) {
		_running = false;
		_cond.notify_all();
	}

	if (_updateThread.joinable())
		_updateThread.join();

	blog_debug("[Utils::Obs::VolumeMeter::Handler::~Handler] Handler destroyed.");
}

void Utils::Obs::VolumeMeter::Handler::UpdateThread()
{
	blog_debug("[Utils::Obs::VolumeMeter::Handler::UpdateThread] Thread started.");
	while (_running) {
		{
			std::unique_lock<std::mutex> l(_mutex);
			if (_cond.wait_for(l, std::chrono::milliseconds(_updatePeriod), [this] { return !_running; }))
				break;
		}

		std::vector<json> inputs;
		std::vector<json> outputs;
		std::unique_lock<std::mutex> l(_meterMutex);
		
		// Input meters 처리
		for (auto &meter : _meters) {
			if (meter->InputValid())
				inputs.push_back(meter->GetMeterData());
		}
		
		// Output meters 처리
		blog_debug("[Handler::UpdateThread] Total output meters: %zu", _outputMeters.size());
		for (auto &outputMeter : _outputMeters) {
			if (outputMeter->OutputValid()) {
				json outputData = outputMeter->GetMeterData();
				outputs.push_back(outputData);
				blog_debug("[Handler::UpdateThread] Added output data for: %s", 
				          outputData.contains("outputName") ? outputData["outputName"].get<std::string>().c_str() : "Unknown");
			} else {
				blog_debug("[Handler::UpdateThread] Output meter is invalid");
			}
		}
		l.unlock();

		blog_debug("[Handler::UpdateThread] Sending callback with %zu inputs, %zu outputs", inputs.size(), outputs.size());
		if (_updateCallback)
			_updateCallback(inputs, outputs);
	}
	blog_debug("[Utils::Obs::VolumeMeter::Handler::UpdateThread] Thread stopped.");
}

void Utils::Obs::VolumeMeter::Handler::InputActivateCallback(void *priv_data, calldata_t *cd)
{
	auto c = static_cast<Handler *>(priv_data);

	obs_source_t *source = GetCalldataPointer<obs_source_t>(cd, "source");
	if (!source)
		return;

	uint32_t flags = obs_source_get_output_flags(source);
	if ((flags & OBS_SOURCE_AUDIO) == 0)
		return;

	std::unique_lock<std::mutex> l(c->_meterMutex);
	
	// Add input sources
	if (obs_source_get_type(source) == OBS_SOURCE_TYPE_INPUT) {
		c->_meters.emplace_back(std::move(new Meter(source)));
	}

	// Add output sources (desktop audio capture sources)
	const char *sourceId = obs_source_get_id(source);
	if (sourceId && (strstr(sourceId, "wasapi_output_capture") || 
	                 strstr(sourceId, "pulse_output_capture") ||
	                 strstr(sourceId, "coreaudio_output_capture"))) {
		c->_outputMeters.emplace_back(std::move(new OutputMeter(source)));
		blog_debug("[Handler::InputActivateCallback] Added output meter for: %s (ID: %s)", 
		          obs_source_get_name(source), sourceId);
	}
}

void Utils::Obs::VolumeMeter::Handler::InputDeactivateCallback(void *priv_data, calldata_t *cd)
{
	auto c = static_cast<Handler *>(priv_data);

	obs_source_t *source = GetCalldataPointer<obs_source_t>(cd, "source");
	if (!source)
		return;

	// Don't ask me why, but using std::remove_if segfaults trying this.
	std::unique_lock<std::mutex> l(c->_meterMutex);
	
	// Remove input meters
	if (obs_source_get_type(source) == OBS_SOURCE_TYPE_INPUT) {
	std::vector<MeterPtr>::iterator iter;
	for (iter = c->_meters.begin(); iter != c->_meters.end();) {
			if (obs_weak_source_references_source(iter->get()->GetWeakInput(), source))
			iter = c->_meters.erase(iter);
		else
			++iter;
		}
	}

	// Remove output meters
	const char *sourceId = obs_source_get_id(source);
	if (sourceId && (strstr(sourceId, "wasapi_output_capture") || 
	                 strstr(sourceId, "pulse_output_capture") ||
	                 strstr(sourceId, "coreaudio_output_capture"))) {
		std::vector<OutputMeterPtr>::iterator iter;
		for (iter = c->_outputMeters.begin(); iter != c->_outputMeters.end();) {
			if (obs_weak_source_references_source(iter->get()->GetWeakOutput(), source)) {
				blog_debug("[Handler::InputDeactivateCallback] Removed output meter for: %s (ID: %s)", 
				          obs_source_get_name(source), sourceId);
				iter = c->_outputMeters.erase(iter);
			} else
				++iter;
		}
	}
}
