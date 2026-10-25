// SPDX-License-Identifier: MIT
/*
 * Userspace IIO Telemetry Reader & Latency/Jitter Benchmark Client
 *
 * Copyright (c) 2026 Nikunj Malik
 *
 * Standard Linux IIO ABI Semantics Handled:
 *  - `in_temp_input`:             milli-degrees Celsius (integer, e.g. "25080" -> 25.080 degC)
 *  - `in_pressure_input`:         kilopascals (kPa) formatted by the kernel IIO core from
 *                                 `IIO_VAL_FRACTIONAL` (`val / val2` = `Q24.8 Pa / 256000`)
 *                                 as a fixed-point decimal string (e.g. "100.652343") or
 *                                 rational fraction ("25766999/256000"). Both representations
 *                                 are parsed and verified.
 *  - `in_humidityrelative_input`: milli-percent relative humidity (integer, e.g. "53952" -> 53.952 %RH)
 *  - `in_sampling_frequency` / `sampling_frequency`: hardware standby rate in Hz (1, 2, 4, 8, 16, 26)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

#define DEFAULT_IIO_BASE_PATH	"/sys/bus/iio/devices"
#define MAX_PATH_LEN		512
#define MAX_BUF_LEN		128

static volatile sig_atomic_t g_keep_running = 1;

static void handle_sigint(int sig)
{
	(void)sig;
	g_keep_running = 0;
}

enum output_format {
	FMT_TEXT = 0,
	FMT_JSON,
	FMT_CSV
};

struct cli_config {
	char sysfs_path[MAX_PATH_LEN];
	char log_file[MAX_PATH_LEN];
	int count;
	int interval_ms;
	int set_freq_hz;
	enum output_format format;
	bool print_stats;
	bool fail_fast;
};

struct sensor_sample {
	uint64_t seq;
	double timestamp_s;
	double temp_c;
	double pressure_kpa;
	double pressure_hpa;
	double humidity_pct;
	int sampling_freq_hz;
	double read_latency_ms;
	double interval_jitter_ms;
};

struct perf_stats {
	uint64_t total_attempts;
	uint64_t successful_reads;
	uint64_t failed_reads;
	int last_errno;

	double min_latency_ms;
	double max_latency_ms;
	double sum_latency_ms;
	double sum_sq_latency_ms;

	double max_abs_jitter_ms;
	double sum_abs_jitter_ms;
	uint64_t jitter_count;
};

static uint64_t monotonic_ns(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
		return 0;
	return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void sleep_until_monotonic(const struct timespec *target)
{
#if defined(TIMER_ABSTIME) && !defined(__APPLE__)
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, target, NULL);
#else
	uint64_t now_ns = monotonic_ns();
	uint64_t target_ns = (uint64_t)target->tv_sec * 1000000000ULL + (uint64_t)target->tv_nsec;
	if (target_ns > now_ns) {
		uint64_t diff_ns = target_ns - now_ns;
		struct timespec req;
		req.tv_sec = (time_t)(diff_ns / 1000000000ULL);
		req.tv_nsec = (long)(diff_ns % 1000000000ULL);
		nanosleep(&req, NULL);
	}
#endif
}

static void trim_whitespace(char *s)
{
	size_t len = strlen(s);

	while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r' ||
			   s[len - 1] == ' '  || s[len - 1] == '\t')) {
		s[len - 1] = '\0';
		len--;
	}
}

/*
 * read_sysfs_str() - Open, read, and close a sysfs attribute atomically.
 * Returns 0 on success or negative errno on failure.
 */
static int read_sysfs_str(const char *dir_path, const char *attr_name,
			  char *out_buf, size_t out_sz)
{
	char full_path[MAX_PATH_LEN];
	int fd;
	ssize_t n;

	snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, attr_name);
	fd = open(full_path, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
		return -errno;

	n = read(fd, out_buf, out_sz - 1);
	if (n < 0) {
		int saved_err = -errno;

		close(fd);
		return saved_err;
	}
	close(fd);

	if (n == 0)
		return -ENODATA;

	out_buf[n] = '\0';
	trim_whitespace(out_buf);
	return 0;
}

/*
 * write_sysfs_int() - Write an integer value to a writable IIO sysfs attribute.
 * Returns 0 on success or negative errno on failure.
 */
static int write_sysfs_int(const char *dir_path, const char *attr_name, int val)
{
	char full_path[MAX_PATH_LEN];
	char buf[64];
	int fd, len;
	ssize_t n;

	snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, attr_name);
	fd = open(full_path, O_WRONLY | O_CLOEXEC);
	if (fd < 0)
		return -errno;

	len = snprintf(buf, sizeof(buf), "%d\n", val);
	n = write(fd, buf, (size_t)len);
	if (n < 0) {
		int saved_err = -errno;

		close(fd);
		return saved_err;
	}
	close(fd);
	return 0;
}

/*
 * parse_iio_number() - Parse integer, kernel-formatted decimal (from IIO_VAL_FRACTIONAL),
 * or explicit rational fraction "numerator/denominator" strings.
 * Rejects trailing garbage or NaN/Inf values.
 */
static int parse_iio_number(const char *str, double *val_out)
{
	const char *slash;
	char *endptr = NULL;

	if (!str || str[0] == '\0')
		return -EINVAL;

	/* Support simulated fault injection token "ERR:-5" */
	if (strncmp(str, "ERR:", 4) == 0) {
		long err_code = strtol(str + 4, &endptr, 10);

		if (err_code < 0)
			return (int)err_code;
		return -EIO;
	}

	slash = strchr(str, '/');
	if (slash) {
		char *num_end = NULL;
		char *den_end = NULL;
		double num, den;

		errno = 0;
		num = strtod(str, &num_end);
		if (errno != 0 || num_end != slash)
			return -EINVAL;

		errno = 0;
		den = strtod(slash + 1, &den_end);
		if (errno != 0 || den_end == slash + 1 || *den_end != '\0')
			return -EINVAL;

		if (fabs(den) < 1e-12 || !isfinite(num) || !isfinite(den))
			return -ERANGE;

		*val_out = num / den;
		return 0;
	}

	errno = 0;
	*val_out = strtod(str, &endptr);
	if (errno != 0 || endptr == str || *endptr != '\0' || !isfinite(*val_out))
		return -EINVAL;

	return 0;
}

/*
 * discover_iio_device() - Scan /sys/bus/iio/devices for bme280_custom or bme280
 */
static int discover_iio_device(const char *base_dir, char *resolved_path, size_t path_sz)
{
	DIR *dir;
	struct dirent *entry;
	char candidate[MAX_PATH_LEN];
	char name_buf[MAX_BUF_LEN];

	dir = opendir(base_dir);
	if (!dir)
		return -errno;

	while ((entry = readdir(dir)) != NULL) {
		if (strncmp(entry->d_name, "iio:device", 10) != 0)
			continue;

		snprintf(candidate, sizeof(candidate), "%s/%s", base_dir, entry->d_name);
		if (read_sysfs_str(candidate, "name", name_buf, sizeof(name_buf)) == 0) {
			if (strcmp(name_buf, "bme280_custom") == 0 ||
			    strcmp(name_buf, "bme280") == 0) {
				snprintf(resolved_path, path_sz, "%s", candidate);
				closedir(dir);
				return 0;
			}
		}
	}

	closedir(dir);
	return -ENODEV;
}

/*
 * read_sampling_frequency() - Read `sampling_frequency` or `in_sampling_frequency`
 */
static int read_sampling_frequency(const char *sysfs_dir, int *freq_out)
{
	char buf[MAX_BUF_LEN];
	double val = 0.0;
	int ret;

	ret = read_sysfs_str(sysfs_dir, "sampling_frequency", buf, sizeof(buf));
	if (ret < 0)
		ret = read_sysfs_str(sysfs_dir, "in_sampling_frequency", buf, sizeof(buf));
	if (ret < 0)
		return ret;

	ret = parse_iio_number(buf, &val);
	if (ret < 0)
		return ret;

	*freq_out = (int)lround(val);
	return 0;
}

/*
 * read_sensor_channels() - Read temperature, pressure, and humidity from IIO sysfs
 */
static int read_sensor_channels(const char *sysfs_dir, struct sensor_sample *sample)
{
	char buf[MAX_BUF_LEN];
	double raw_temp_milli = 0.0;
	double raw_press_kpa = 0.0;
	double raw_hum_milli = 0.0;
	int freq_hz = 0;
	int ret;

	ret = read_sysfs_str(sysfs_dir, "in_temp_input", buf, sizeof(buf));
	if (ret < 0)
		return ret;
	ret = parse_iio_number(buf, &raw_temp_milli);
	if (ret < 0)
		return ret;

	ret = read_sysfs_str(sysfs_dir, "in_pressure_input", buf, sizeof(buf));
	if (ret < 0)
		return ret;
	ret = parse_iio_number(buf, &raw_press_kpa);
	if (ret < 0)
		return ret;

	ret = read_sysfs_str(sysfs_dir, "in_humidityrelative_input", buf, sizeof(buf));
	if (ret < 0)
		return ret;
	ret = parse_iio_number(buf, &raw_hum_milli);
	if (ret < 0)
		return ret;

	if (read_sampling_frequency(sysfs_dir, &freq_hz) == 0)
		sample->sampling_freq_hz = freq_hz;

	sample->temp_c = raw_temp_milli / 1000.0;
	sample->pressure_kpa = raw_press_kpa;
	sample->pressure_hpa = raw_press_kpa * 10.0;
	sample->humidity_pct = raw_hum_milli / 1000.0;

	/* Physical plausibility sanity check (Bosch BME280 operating range) */
	if (sample->temp_c < -45.0 || sample->temp_c > 90.0 ||
	    sample->pressure_hpa < 250.0 || sample->pressure_hpa > 1200.0 ||
	    sample->humidity_pct < 0.0 || sample->humidity_pct > 105.0) {
		return -ERANGE;
	}

	return 0;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [OPTIONS]\n\n", prog);
	printf("Options:\n");
	printf("  -p, --sysfs-path <path>   Explicit IIO device path (default: auto-discover in %s)\n",
	       DEFAULT_IIO_BASE_PATH);
	printf("  -c, --count <N>           Number of samples to read (0 = continuous, default: 1)\n");
	printf("  -i, --interval-ms <ms>    Polling interval in milliseconds (default: 100)\n");
	printf("  -S, --set-freq <Hz>       Program hardware sampling_frequency (1, 2, 4, 8, 16, 26 Hz)\n");
	printf("  -f, --format <fmt>        Output format: text | json | csv (default: text)\n");
	printf("  -o, --output <file>       Append structured readings to file\n");
	printf("  -s, --stats               Print latency and jitter summary report on exit\n");
	printf("  -F, --fail-fast           Exit immediately with non-zero code on first read error\n");
	printf("  -h, --help                Show this help message\n");
}

int main(int argc, char **argv)
{
	struct cli_config cfg = {
		.sysfs_path = "",
		.log_file = "",
		.count = 1,
		.interval_ms = 100,
		.set_freq_hz = 0,
		.format = FMT_TEXT,
		.print_stats = false,
		.fail_fast = false,
	};
	struct perf_stats stats = {
		.min_latency_ms = 1e9,
		.max_latency_ms = 0.0,
	};
	FILE *log_fp = NULL;
	int opt, ret;

	static const struct option long_opts[] = {
		{ "sysfs-path",  required_argument, NULL, 'p' },
		{ "count",       required_argument, NULL, 'c' },
		{ "interval-ms", required_argument, NULL, 'i' },
		{ "set-freq",    required_argument, NULL, 'S' },
		{ "format",      required_argument, NULL, 'f' },
		{ "output",      required_argument, NULL, 'o' },
		{ "stats",       no_argument,       NULL, 's' },
		{ "fail-fast",   no_argument,       NULL, 'F' },
		{ "help",        no_argument,       NULL, 'h' },
		{ NULL,          0,                 NULL, 0   }
	};

	while ((opt = getopt_long(argc, argv, "p:c:i:S:f:o:sFh", long_opts, NULL)) != -1) {
		switch (opt) {
		case 'p':
			snprintf(cfg.sysfs_path, sizeof(cfg.sysfs_path), "%s", optarg);
			break;
		case 'c':
			cfg.count = atoi(optarg);
			break;
		case 'i':
			cfg.interval_ms = atoi(optarg);
			if (cfg.interval_ms < 1)
				cfg.interval_ms = 1;
			break;
		case 'S':
			cfg.set_freq_hz = atoi(optarg);
			break;
		case 'f':
			if (strcmp(optarg, "json") == 0)
				cfg.format = FMT_JSON;
			else if (strcmp(optarg, "csv") == 0)
				cfg.format = FMT_CSV;
			else
				cfg.format = FMT_TEXT;
			break;
		case 'o':
			snprintf(cfg.log_file, sizeof(cfg.log_file), "%s", optarg);
			break;
		case 's':
			cfg.print_stats = true;
			break;
		case 'F':
			cfg.fail_fast = true;
			break;
		case 'h':
			print_usage(argv[0]);
			return EXIT_SUCCESS;
		default:
			print_usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	signal(SIGINT, handle_sigint);
	signal(SIGTERM, handle_sigint);

	/* Resolve IIO device path if not explicitly supplied */
	if (cfg.sysfs_path[0] == '\0') {
		ret = discover_iio_device(DEFAULT_IIO_BASE_PATH,
					  cfg.sysfs_path, sizeof(cfg.sysfs_path));
		if (ret < 0) {
			fprintf(stderr,
				"[ERROR] Failed to discover BME280 IIO device under %s: %s (%d)\n",
				DEFAULT_IIO_BASE_PATH, strerror(-ret), ret);
			return EXIT_FAILURE;
		}
	} else {
		/* Verify that the provided sysfs path exists and contains 'name' */
		char dev_name[MAX_BUF_LEN];

		ret = read_sysfs_str(cfg.sysfs_path, "name", dev_name, sizeof(dev_name));
		if (ret < 0) {
			fprintf(stderr,
				"[ERROR] Cannot access IIO device at '%s': %s (%d)\n",
				cfg.sysfs_path, strerror(-ret), ret);
			return EXIT_FAILURE;
		}
	}

	/* Optional: reprogram hardware sampling_frequency via sysfs */
	if (cfg.set_freq_hz > 0) {
		ret = write_sysfs_int(cfg.sysfs_path, "sampling_frequency", cfg.set_freq_hz);
		if (ret < 0)
			ret = write_sysfs_int(cfg.sysfs_path, "in_sampling_frequency", cfg.set_freq_hz);
		if (ret < 0) {
			fprintf(stderr,
				"[ERROR] Failed to set sampling_frequency=%d Hz on %s: %s (%d)\n",
				cfg.set_freq_hz, cfg.sysfs_path, strerror(-ret), ret);
			return EXIT_FAILURE;
		}
	}

	if (cfg.log_file[0] != '\0') {
		log_fp = fopen(cfg.log_file, "a");
		if (!log_fp) {
			fprintf(stderr, "[ERROR] Cannot open log file '%s': %s\n",
				cfg.log_file, strerror(errno));
			return EXIT_FAILURE;
		}
	}

	if (cfg.format == FMT_CSV) {
		printf("seq,timestamp_s,temp_c,pressure_kpa,pressure_hpa,humidity_pct,sampling_freq_hz,latency_ms,jitter_ms\n");
		if (log_fp)
			fprintf(log_fp, "seq,timestamp_s,temp_c,pressure_kpa,pressure_hpa,humidity_pct,sampling_freq_hz,latency_ms,jitter_ms\n");
	}

	uint64_t start_epoch_ns = monotonic_ns();
	uint64_t prev_sample_start_ns = 0;
	struct timespec next_wakeup;
	clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

	int iteration = 0;
	while (g_keep_running && (cfg.count == 0 || iteration < cfg.count)) {
		struct sensor_sample sample;
		memset(&sample, 0, sizeof(sample));

		uint64_t t0_ns = monotonic_ns();
		if (prev_sample_start_ns > 0) {
			double actual_dt_ms = (double)(t0_ns - prev_sample_start_ns) / 1e6;
			sample.interval_jitter_ms = actual_dt_ms - (double)cfg.interval_ms;
			double abs_jitter = fabs(sample.interval_jitter_ms);
			stats.sum_abs_jitter_ms += abs_jitter;
			if (abs_jitter > stats.max_abs_jitter_ms)
				stats.max_abs_jitter_ms = abs_jitter;
			stats.jitter_count++;
		}
		prev_sample_start_ns = t0_ns;

		stats.total_attempts++;
		ret = read_sensor_channels(cfg.sysfs_path, &sample);
		uint64_t t1_ns = monotonic_ns();

		sample.seq = stats.total_attempts;
		sample.timestamp_s = (double)(t1_ns - start_epoch_ns) / 1e9;
		sample.read_latency_ms = (double)(t1_ns - t0_ns) / 1e6;

		if (ret < 0) {
			stats.failed_reads++;
			stats.last_errno = -ret;
			fprintf(stderr,
				"[ERROR] Sample #%llu read failed on %s: %s (errno=%d)\n",
				(unsigned long long)sample.seq, cfg.sysfs_path,
				strerror(-ret), -ret);
			if (cfg.fail_fast) {
				if (log_fp)
					fclose(log_fp);
				return EXIT_FAILURE;
			}
		} else {
			stats.successful_reads++;
			if (sample.read_latency_ms < stats.min_latency_ms)
				stats.min_latency_ms = sample.read_latency_ms;
			if (sample.read_latency_ms > stats.max_latency_ms)
				stats.max_latency_ms = sample.read_latency_ms;
			stats.sum_latency_ms += sample.read_latency_ms;
			stats.sum_sq_latency_ms += sample.read_latency_ms * sample.read_latency_ms;

			if (cfg.format == FMT_JSON) {
				printf("{\"seq\":%llu,\"timestamp_s\":%.6f,\"temp_c\":%.3f,"
				       "\"pressure_kpa\":%.4f,\"pressure_hpa\":%.3f,"
				       "\"humidity_pct\":%.3f,\"sampling_freq_hz\":%d,"
				       "\"latency_ms\":%.3f,\"jitter_ms\":%.3f}\n",
				       (unsigned long long)sample.seq, sample.timestamp_s,
				       sample.temp_c, sample.pressure_kpa, sample.pressure_hpa,
				       sample.humidity_pct, sample.sampling_freq_hz,
				       sample.read_latency_ms, sample.interval_jitter_ms);
				if (log_fp) {
					fprintf(log_fp,
						"{\"seq\":%llu,\"timestamp_s\":%.6f,\"temp_c\":%.3f,"
						"\"pressure_kpa\":%.4f,\"pressure_hpa\":%.3f,"
						"\"humidity_pct\":%.3f,\"sampling_freq_hz\":%d,"
						"\"latency_ms\":%.3f,\"jitter_ms\":%.3f}\n",
						(unsigned long long)sample.seq, sample.timestamp_s,
						sample.temp_c, sample.pressure_kpa, sample.pressure_hpa,
						sample.humidity_pct, sample.sampling_freq_hz,
						sample.read_latency_ms, sample.interval_jitter_ms);
				}
			} else if (cfg.format == FMT_CSV) {
				printf("%llu,%.6f,%.3f,%.4f,%.3f,%.3f,%d,%.3f,%.3f\n",
				       (unsigned long long)sample.seq, sample.timestamp_s,
				       sample.temp_c, sample.pressure_kpa, sample.pressure_hpa,
				       sample.humidity_pct, sample.sampling_freq_hz,
				       sample.read_latency_ms, sample.interval_jitter_ms);
				if (log_fp) {
					fprintf(log_fp, "%llu,%.6f,%.3f,%.4f,%.3f,%.3f,%d,%.3f,%.3f\n",
						(unsigned long long)sample.seq, sample.timestamp_s,
						sample.temp_c, sample.pressure_kpa, sample.pressure_hpa,
						sample.humidity_pct, sample.sampling_freq_hz,
						sample.read_latency_ms, sample.interval_jitter_ms);
				}
			} else {
				printf("[%8.3f s] #%04llu | Temp: %6.2f C | Press: %7.2f hPa (%6.3f kPa) | "
				       "Hum: %5.2f %%RH | Freq: %2d Hz | Latency: %5.3f ms | Jitter: %+6.3f ms\n",
				       sample.timestamp_s, (unsigned long long)sample.seq,
				       sample.temp_c, sample.pressure_hpa, sample.pressure_kpa,
				       sample.humidity_pct, sample.sampling_freq_hz,
				       sample.read_latency_ms, sample.interval_jitter_ms);
			}
			fflush(stdout);
		}

		iteration++;
		if (cfg.count != 0 && iteration >= cfg.count)
			break;

		/* Advance monotonic target wakeup time to prevent cumulative drift */
		long add_ns = (long)cfg.interval_ms * 1000000L;
		next_wakeup.tv_nsec += add_ns;
		while (next_wakeup.tv_nsec >= 1000000000L) {
			next_wakeup.tv_sec += 1;
			next_wakeup.tv_nsec -= 1000000000L;
		}
		sleep_until_monotonic(&next_wakeup);
	}

	if (log_fp)
		fclose(log_fp);

	if (cfg.print_stats && stats.successful_reads > 0) {
		double mean_lat = stats.sum_latency_ms / (double)stats.successful_reads;
		double variance = (stats.sum_sq_latency_ms / (double)stats.successful_reads) -
				  (mean_lat * mean_lat);
		double stddev_lat = variance > 0.0 ? sqrt(variance) : 0.0;
		double mean_jitter = stats.jitter_count > 0 ?
				     (stats.sum_abs_jitter_ms / (double)stats.jitter_count) : 0.0;

		fprintf(stderr,
			"\n=== Performance & Reliability Summary ===\n"
			"Device Path      : %s\n"
			"Total Attempts   : %llu (Success: %llu, Failed: %llu)\n"
			"Read Latency (ms): min=%.3f, mean=%.3f, max=%.3f, stddev=%.3f\n"
			"Polling Jitter   : mean_abs=%.3f ms, max_abs=%.3f ms (target=%d ms)\n",
			cfg.sysfs_path,
			(unsigned long long)stats.total_attempts,
			(unsigned long long)stats.successful_reads,
			(unsigned long long)stats.failed_reads,
			stats.min_latency_ms, mean_lat, stats.max_latency_ms, stddev_lat,
			mean_jitter, stats.max_abs_jitter_ms, cfg.interval_ms);
	}

	if (stats.successful_reads == 0)
		return EXIT_FAILURE;

	return (stats.failed_reads > 0 && cfg.fail_fast) ? EXIT_FAILURE : EXIT_SUCCESS;
}
