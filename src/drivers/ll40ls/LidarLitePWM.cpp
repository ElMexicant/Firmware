/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file LidarLitePWM.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */

#include "LidarLitePWM.h"
#include <stdio.h>
#include <string.h>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_input.h>

#define SEC2USEC		1000000.0f
#define ACC_UNC			0.1f
#define LIDAR_UNC		0.1f
#define POS_UNC_INIT	0.1f
#define VEL_UNC_INIT	0.2f

#include <systemlib/mavlink_log.h> //rickyremove
extern orb_advert_t mavlink_log_pub;//rickyremove

namespace ll40ls {

LidarLitePWM::LidarLitePWM(const char *path) :
	CDev("LidarLitePWM", path),
	_work{},
	_reports(nullptr),
	_class_instance(-1),
	_orb_class_instance(-1),
	_pwmSub(-1),
	_sensorCombinedSub(-1),
	_estimator_initialized(false), // make this true
	_sensorCombined_valid(false), // make this true
	_last_predict(0),
	_pwm{},
	_distance_sensor_topic(nullptr),
	_range{},
	_sample_perf(perf_alloc(PC_ELAPSED, "ll40ls_pwm_read")),
	_read_errors(perf_alloc(PC_COUNT, "ll40ls_pwm_read_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ll40ls_pwm_buffer_overflows")),
	_sensor_zero_resets(perf_alloc(PC_COUNT, "ll40ls_pwm_zero_resets"))
{
}

LidarLitePWM::~LidarLitePWM()
{
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_buffer_overflows);
	perf_free(_sensor_zero_resets);
}


int LidarLitePWM::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != PX4_OK) {
		return PX4_ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(struct distance_sensor_s));

	if (_reports == nullptr) {
		return PX4_ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the distance_sensor topic */
	struct distance_sensor_s ds_report = {};
	measure();
	_reports->get(&ds_report);
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		DEVICE_DEBUG("failed to create distance_sensor object. Did you start uOrb?");
	}

	return PX4_OK;
}

void LidarLitePWM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_read_errors);
	perf_print_counter(_buffer_overflows);
	perf_print_counter(_sensor_zero_resets);
	warnx("poll interval:  %u ticks", getMeasureTicks());
	warnx("distance: %.3fm", (double)_range.current_distance);
}

void LidarLitePWM::print_registers()
{
	printf("Not supported in PWM mode\n");
}

void LidarLitePWM::start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&LidarLitePWM::cycle_trampoline, this, 1);
}

void LidarLitePWM::stop()
{
	work_cancel(HPWORK, &_work);
}

void LidarLitePWM::cycle_trampoline(void *arg)
{
	LidarLitePWM *dev = (LidarLitePWM *)arg;

	dev->cycle();
}

void LidarLitePWM::cycle()
{
	measure();

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&LidarLitePWM::cycle_trampoline,
		   this,
		   getMeasureTicks());
	return;
}

int LidarLitePWM::measure()
{
	perf_begin(_sample_perf);

	if (PX4_OK != collect()) {
		DEVICE_DEBUG("collection error");
		perf_count(_read_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	if (_estimator_initialized) {
		_update_topics(); //rickyremove
		// predict
		// only run prediction if filter has been initialized
		// use the best estimate of the vehicle acceleration to predict the lidar position

		float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

		// predict agl position with the help of accel data
		float acc;

		if (_sensorCombined_valid) {
			acc = _sensorCombined.accelerometer_m_s2[2] + 9.80665f;
			if (acc < .07f || acc > -.07f) {
				acc = 0.0f;
			}
			_sensorCombined_valid = false;
		} else {
			acc = 0.0f;
		}
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lidar] %4.4f", double(acc));
		_kalman_filter.predict(dt, -acc, ACC_UNC);

		_last_predict = hrt_absolute_time();
	}

	if (!_estimator_initialized) {
		// too long since last measurement, reset filter

		_initialize_topics();//rickyremove to elsewhere

		PX4_WARN("Init");
		_kalman_filter.init(0, 0, POS_UNC_INIT, VEL_UNC_INIT);

		_estimator_initialized = true;
		_last_predict = hrt_absolute_time();
	} else {
		_kalman_filter.update(float(_pwm.pulse_width) * 1e-3f, POS_UNC_INIT);

		_range.timestamp = hrt_absolute_time();
		float z, zvel, covz, covz_v;
		_kalman_filter.getState(z, zvel);
		_kalman_filter.getCovariance(covz, covz_v);

		_range.timestamp = hrt_absolute_time();
		_range.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		_range.max_distance = get_maximum_distance();
		_range.min_distance = get_minimum_distance();
		_range.previous_distance = _range.current_distance;
		_range.current_distance = z;
		_range.covariance = covz;
		_range.orientation = 8;
		_range.dv = -zvel;
			//(_range.previous_distance - _range.current_distance) / .05f;
		/* TODO: set proper ID */
		_range.id = 0;
	}

	/* Due to a bug in older versions of the LidarLite firmware, we have to reset sensor on (distance == 0) */
	if (_range.current_distance <= 0.0f) {
		perf_count(_sensor_zero_resets);
		perf_end(_sample_perf);
		return reset_sensor();
	}

	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &_range);
	}

	if (_reports->force(&_range)) {
		perf_count(_buffer_overflows);
	}

	poll_notify(POLLIN);
	perf_end(_sample_perf);
	return PX4_OK;
}

ssize_t LidarLitePWM::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (getMeasureTicks() > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;

	} else {

		_reports->flush();
		measure();

		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}
	}

	return ret;
}

int
LidarLitePWM::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	/* no custom ioctls implemented for now */
	default:
		/* give it to the superclass */
		return LidarLite::ioctl(filp, cmd, arg);
	}
}

int LidarLitePWM::collect()
{
	int fd = ::open(PWMIN0_DEVICE_PATH, O_RDONLY);

	if (fd == -1) {
		return PX4_ERROR;
	}

	if (::read(fd, &_pwm, sizeof(_pwm)) == sizeof(_pwm)) {
		::close(fd);
		return PX4_OK;
	}

	::close(fd);
	return EAGAIN;
}

int LidarLitePWM::reset_sensor()
{

	int fd = ::open(PWMIN0_DEVICE_PATH, O_RDONLY);

	if (fd == -1) {
		return PX4_ERROR;
	}

	int ret = ::ioctl(fd, SENSORIOCRESET, 0);
	::close(fd);
	return ret;
}

void LidarLitePWM::_initialize_topics()
{
	_sensorCombinedSub = orb_subscribe(ORB_ID(sensor_combined));
}

void LidarLitePWM::_update_topics()
{
	_sensorCombined_valid = _orb_update(ORB_ID(sensor_combined), _sensorCombinedSub, &_sensorCombined);
}


bool LidarLitePWM::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

}// namespace ll40ls
