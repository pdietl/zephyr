/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Additional test case for log core
 *
 */

#include <tc_util.h>
#include <stdbool.h>
#include <zephyr/zephyr.h>
#include <ztest.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_backend_std.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_output.h>

#define LOG_MODULE_NAME log_test
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);
static K_SEM_DEFINE(log_sem, 0, 1);

#define TIMESTAMP_FREC (2000000)
ZTEST_BMEM uint32_t source_id;
/* used when log_msg2 create in user space */
ZTEST_BMEM uint8_t domain, level;
ZTEST_DMEM uint32_t msg_data = 0x1234;
ZTEST_DMEM char *test_msg_usr = "test msg";

static uint8_t buf;
static int char_out(uint8_t *data, size_t length, void *ctx)
{
	ARG_UNUSED(data);
	ARG_UNUSED(ctx);
	return length;
}
LOG_OUTPUT_DEFINE(log_output, char_out, &buf, 1);

struct backend_cb {
	/* count log messages handled by this backend */
	size_t counter;
	/* count log messages handled immediately by this backend */
	size_t sync;
	/* inform put() to check timestamp of message against exp_timestamps */
	bool check_timestamp;
	uint32_t exp_timestamps[16];
	/* inform put() to check severity of message against exp_severity */
	bool check_severity;
	uint16_t exp_severity[4];
	/* inform put() to check domain_id of message */
	bool check_domain_id;
	/* How many messages have been logged.
	 * used in async mode, to make sure all logs have been handled by compare
	 * counter with total_logs
	 */
	size_t total_logs;
};

static void process(const struct log_backend *const backend,
		union log_msg2_generic *msg)
{
	uint32_t flags;
	struct backend_cb *cb = (struct backend_cb *)backend->cb->ctx;

	if (IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE)) {
		cb->sync++;
	}

	if (cb->check_domain_id) {
		zassert_equal(log_msg2_get_domain(&(msg->log)), CONFIG_LOG_DOMAIN_ID,
				"Unexpected domain id");
	}

	if (cb->check_timestamp) {
		uint32_t exp_timestamp = cb->exp_timestamps[cb->counter];

		zassert_equal(log_msg2_get_timestamp(&(msg->log)), exp_timestamp,
			      "Unexpected message index");
	}

	if (cb->check_severity) {
		zassert_equal(log_msg2_get_level(&(msg->log)),
			      cb->exp_severity[cb->counter],
			      "Unexpected log severity");
	}

	cb->counter++;
	if (IS_ENABLED(CONFIG_LOG_PROCESS_THREAD)) {
		if (cb->counter == cb->total_logs) {
			k_sem_give(&log_sem);
		}
	}

	if (k_is_user_context()) {
		zassert_equal(log_msg2_get_domain(&(msg->log)), domain,
				"Unexpected domain id");

		zassert_equal(log_msg2_get_level(&(msg->log)), level,
			      "Unexpected log severity");
	}

	flags = log_backend_std_get_flags();
	log_output_msg2_process(&log_output, &msg->log, flags);
}

static void panic(const struct log_backend *const backend)
{
	ARG_UNUSED(backend);
}

const struct log_backend_api log_backend_test_api = {
	.process = process,
	.panic = panic,
};

LOG_BACKEND_DEFINE(backend1, log_backend_test_api, false);
struct backend_cb backend1_cb;

LOG_BACKEND_DEFINE(backend2, log_backend_test_api, false);
struct backend_cb backend2_cb;

/* The logging system support user customize timestamping in log messages
 * by register a timestamp function, in timestamp_get() below, just return
 * a counter as timestamp for different messages.
 * when install this timestamp function, timestamping frequency is set to
 * 2000000, means 2 timestamp/us
 */
static uint32_t stamp;
static uint32_t timestamp_get(void)
{
	stamp++;
	return log_output_timestamp_to_us(stamp * 2);
}

static void log_setup(bool backend2_enable)
{
	stamp = 0U;

	log_init();
#ifndef CONFIG_LOG_PROCESS_THREAD
	log_thread_set(k_current_get());
#endif

	memset(&backend1_cb, 0, sizeof(backend1_cb));

	log_backend_enable(&backend1, &backend1_cb, LOG_LEVEL_DBG);

	if (backend2_enable) {
		memset(&backend2_cb, 0, sizeof(backend2_cb));

		log_backend_enable(&backend2, &backend2_cb, LOG_LEVEL_DBG);
	} else {
		log_backend_disable(&backend2);
	}
}

static bool log_test_process(bool bypass)
{
	if (IS_ENABLED(CONFIG_LOG_PROCESS_THREAD)) {
		/* waiting for all logs have been handled */
		k_sem_take(&log_sem, K_FOREVER);
		return false;
	} else {
		return log_process(bypass);
	}
}

/**
 * @brief Support multi-processor systems
 *
 * @details Logging system identify domain/processor by domain_id which is now
 *          statically configured by CONFIG_LOG_DOMAIN_ID
 *
 * @addtogroup logging
 */

void test_log_domain_id(void)
{
	log_setup(false);

	backend1_cb.check_domain_id = true;
	backend1_cb.total_logs = 1;

	LOG_INF("info message for domain id test");

	while (log_test_process(false)) {
	}

	zassert_equal(backend1_cb.total_logs, backend1_cb.counter,
		      "Unexpected amount of messages received by the backend");
}

/**
 * @brief Synchronous processing of logging messages.
 *
 * @details if CONFIG_LOG_MODE_IMMEDIATE is enabled, log message is
 *          handled immediately
 *
 * @addtogroup logging
 */

void test_log_sync(void)
{
	TC_PRINT("Logging synchronously\n");

	if (IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE)) {
		log_setup(false);
		LOG_INF("Log immediately");
		LOG_INF("Log immediately");

		/* log immediately, no log_process needed */
		zassert_equal(2, backend1_cb.sync,
			      "Unexpected amount of messages received by the backend.");
	} else {
		ztest_test_skip();
	}
}

/**
 * @brief Early logging
 * @details Handle log message attempts as well as creating new log contexts
 *         instance, before the backend are active
 *
 * @addtogroup logging
 */

void test_log_early_logging(void)
{
	if (IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE)) {
		ztest_test_skip();
	} else {
		log_init();

		/* deactivate other backends */
		const struct log_backend *backend;

		for (int i = 0; i < log_backend_count_get(); i++) {
			backend = log_backend_get(i);
			if (strcmp(backend->name, "test")) {
				log_backend_deactivate(backend);
			}
		}

		TC_PRINT("Create log message before backend active\n");

		LOG_INF("log info before backend active");
		LOG_WRN("log warn before backend active");
		LOG_ERR("log error before backend active");

		TC_PRINT("Activate backend with context");
		memset(&backend1_cb, 0, sizeof(backend1_cb));
		backend1_cb.total_logs = 3;
		log_backend_enable(&backend1, &backend1_cb, LOG_LEVEL_DBG);

		while (log_test_process(false)) {
		}

		zassert_equal(backend1_cb.total_logs, backend1_cb.counter,
			      "Unexpected amount of messages received. %d",
			      backend1_cb.counter);
	}
}

/**
 * @brief Log severity
 *
 * @details This module is registered with LOG_LEVEL_INF, LOG_LEVEL_DBG will be
 *          filtered out at compile time, only 3 message handled
 *
 * @addtogroup logging
 */

void test_log_severity(void)
{
	log_setup(false);

	backend1_cb.check_severity = true;
	backend1_cb.exp_severity[0] = LOG_LEVEL_INF;
	backend1_cb.exp_severity[1] = LOG_LEVEL_WRN;
	backend1_cb.exp_severity[2] = LOG_LEVEL_ERR;

	LOG_INF("info message");
	LOG_WRN("warning message");
	LOG_ERR("error message");
	backend1_cb.total_logs = 3;

	while (log_test_process(false)) {
	}

	zassert_equal(backend1_cb.total_logs, backend1_cb.counter,
		      "Unexpected amount of messages received by the backend.");
}

/**
 * @brief Customizable timestamping in log messages
 *
 * @details Log core permit user to register customized timestamp function
 *
 * @addtogroup logging
 */

void test_log_timestamping(void)
{
	stamp = 0U;

	log_init();
	/* deactivate all other backend */
	const struct log_backend *backend;

	for (int i = 0; i < log_backend_count_get(); i++) {
		backend = log_backend_get(i);
		log_backend_deactivate(backend);
	}

	TC_PRINT("Register timestamp function\n");
	zassert_equal(-EINVAL, log_set_timestamp_func(NULL, 0),
		      "Expects successful timestamp function setting.");
	zassert_equal(0, log_set_timestamp_func(timestamp_get, TIMESTAMP_FREC),
		      "Expects successful timestamp function setting.");

	memset(&backend1_cb, 0, sizeof(backend1_cb));
	log_backend_enable(&backend1, &backend1_cb, LOG_LEVEL_DBG);

	backend1_cb.check_timestamp = true;

	backend1_cb.exp_timestamps[0] = 1U;
	backend1_cb.exp_timestamps[1] = 2U;
	backend1_cb.exp_timestamps[2] = 3U;

	LOG_INF("test timestamp");
	LOG_INF("test timestamp");
	LOG_WRN("test timestamp");
	backend1_cb.total_logs = 3;

	while (log_test_process(false)) {
	}

	zassert_equal(backend1_cb.total_logs,
		      backend1_cb.counter,
		      "Unexpected amount of messages received by the backend.");
}

/**
 * @brief Multiple logging backends
 *
 * @details Enable two backends in this module and enable UART backend
 *          by CONFIG_LOG_BACKEND_UART, there are three backends at least.
 *
 * @addtogroup logging
 */

#define UART_BACKEND "log_backend_uart"
void test_multiple_backends(void)
{
	TC_PRINT("Test multiple backends");
	/* enable both backend1 and backend2 */
	log_setup(true);
	zassert_true((log_backend_count_get() >= 2),
		     "There is no multi backends");

	if (IS_ENABLED(CONFIG_LOG_BACKEND_UART)) {
		bool have_uart = false;
		struct log_backend const *backend;

		for (int i = 0; i < log_backend_count_get(); i++) {
			backend = log_backend_get(i);
			if (strcmp(backend->name, UART_BACKEND) == 0) {
				have_uart = true;
			}
		}
		zassert_true(have_uart, "There is no UART log backend found");
	}
}

/**
 * @brief Process all logging activities using a dedicated thread
 *
 * @addtogroup logging
 */

#ifdef CONFIG_LOG_PROCESS_THREAD
void test_log_thread(void)
{
	uint32_t slabs_free, used, max;

	TC_PRINT("Logging buffer is configured to %d bytes\n",
		 CONFIG_LOG_BUFFER_SIZE);

	TC_PRINT("Stack size of logging thread is configured by ");
	TC_PRINT("CONFIG_LOG_PROCESS_THREAD_STACK_SIZE: %d bytes\n",
		 CONFIG_LOG_PROCESS_THREAD_STACK_SIZE);

	log_setup(false);

	slabs_free = log_msg_mem_get_free();
	used = log_msg_mem_get_used();
	max = log_msg_mem_get_max_used();
	zassert_equal(used, 0, NULL);

	LOG_INF("log info to log thread");
	LOG_WRN("log warning to log thread");
	LOG_ERR("log error to log thread");

	zassert_equal(log_msg_mem_get_used(), 3, NULL);
	zassert_equal(log_msg_mem_get_free(), slabs_free - 3, NULL);
	zassert_equal(log_msg_mem_get_max_used(), max, NULL);

	TC_PRINT("after log, free: %d, used: %d, max: %d\n", slabs_free, used, max);
	/* wait 2 seconds for logging thread to handle this log message*/
	k_sleep(K_MSEC(2000));
	zassert_equal(3, backend1_cb.counter,
		      "Unexpected amount of messages received by the backend.");
	zassert_equal(log_msg_mem_get_used(), 0, NULL);
}
#else
void test_log_thread(void)
{
	ztest_test_skip();
}
#endif

static void call_log_generic(uint32_t source_id, const char *fmt, ...)
{
	struct log_msg_ids src_level = {
		.level = LOG_LEVEL_INF,
		.domain_id = CONFIG_LOG_DOMAIN_ID,
		.source_id = source_id,
	};

	va_list ap;

	va_start(ap, fmt);
	log_generic(src_level, fmt, ap, LOG_STRDUP_EXEC);
	va_end(ap);
}

void test_log_generic(void)
{
	source_id = LOG_CURRENT_MODULE_ID();
	char *log_msg = "log user space";

	log_setup(false);
	backend1_cb.total_logs = 4;

	call_log_generic(source_id, "log generic");
	call_log_generic(source_id, "log generic: %s", log_msg);
	call_log_generic(source_id, "log generic %d\n", source_id);
	call_log_generic(source_id, "log generic %d, %d\n", source_id, 1);
	while (log_test_process(false)) {
	}
}

void test_log_msg2_create(void)
{
	log_setup(false);
	if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED)) {
		int mode;

		domain = 3;
		level = 2;

		z_log_msg2_runtime_create(domain, __log_current_const_data,
					  level, &msg_data, 0,
					  sizeof(msg_data), NULL);
		/* try z_log_msg2_static_create() */
		Z_LOG_MSG2_STACK_CREATE(0, domain, __log_current_const_data,
					level, &msg_data,
					sizeof(msg_data), NULL);

		Z_LOG_MSG2_CREATE(!IS_ENABLED(CONFIG_USERSPACE), mode,
			  CONFIG_LOG_DOMAIN_ID, NULL,
			  LOG_LEVEL_INTERNAL_RAW_STRING, NULL, 0, test_msg_usr);

		while (log_test_process(false)) {
		}
	}
}

void test_log_msg2_create_user(void)
{
	int mode;

	domain = 3;
	level = 2;

	z_log_msg2_runtime_create(domain, NULL,
				  level, &msg_data, 0,
				  sizeof(msg_data), test_msg_usr);
	/* try z_log_msg2_static_create() */
	Z_LOG_MSG2_STACK_CREATE(0, domain, NULL,
				level, &msg_data,
				sizeof(msg_data), test_msg_usr);

	Z_LOG_MSG2_CREATE(!IS_ENABLED(CONFIG_USERSPACE), mode,
		  CONFIG_LOG_DOMAIN_ID, NULL,
		  LOG_LEVEL_INTERNAL_RAW_STRING, NULL, 0, test_msg_usr);

	while (log_test_process(false)) {
	}
}
/* The log process thread has the K_LOWEST_APPLICATION_THREAD_PRIO, adjust it
 * to a higher priority to increase the chances of being scheduled to handle
 * log message as soon as possible
 */
void promote_log_thread(const struct k_thread *thread, void *user_data)
{
	if (!(strcmp(thread->name, "logging"))) {
		k_thread_priority_set((k_tid_t)thread, -1);
	}
}

extern void test_log_from_user(void);
extern void test_log_hexdump_from_user(void);
extern void test_log_generic_user(void);
extern void test_log_filter_set(void);
extern void test_log_panic(void);

/*test case main entry*/
void test_main(void)
{
#ifdef CONFIG_LOG_PROCESS_THREAD
	k_thread_foreach(promote_log_thread, NULL);
#endif

#ifdef CONFIG_USERSPACE
	ztest_test_suite(test_log_core_additional,
			 ztest_user_unit_test(test_log_from_user),
			 ztest_user_unit_test(test_log_hexdump_from_user),
			 ztest_user_unit_test(test_log_generic_user),
			 ztest_user_unit_test(test_log_filter_set),
			 ztest_user_unit_test(test_log_panic),
			 ztest_user_unit_test(test_log_msg2_create_user));
	ztest_run_test_suite(test_log_core_additional);
#else
	ztest_test_suite(test_log_core_additional,
			 ztest_unit_test(test_multiple_backends),
			 ztest_unit_test(test_log_generic),
			 ztest_unit_test(test_log_domain_id),
			 ztest_unit_test(test_log_severity),
			 ztest_unit_test(test_log_timestamping),
			 ztest_unit_test(test_log_early_logging),
			 ztest_unit_test(test_log_sync),
			 ztest_unit_test(test_log_thread),
			 ztest_unit_test(test_log_msg2_create)
			 );
	ztest_run_test_suite(test_log_core_additional);
#endif
}
