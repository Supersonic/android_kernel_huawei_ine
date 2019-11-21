/* hisi_cpufreq_lowtemp.h */

#ifndef __HISI_CPUFREQ_LOWTEMP_H__
#define __HISI_CPUFREQ_LOWTEMP_H__

#ifdef CONFIG_HISI_CPUFREQ_LOWTEMP
/*return true if low temperature, false if not*/
bool is_low_temprature(void);

/*can only be called by fingerprint module
 *request little cluster frequency to high level in low temerature state,
 *little cluster frequency will recovery to low freq after timeout_ms
 *return: 0 if success
 */
int cpufreq_lowtemp_request(const unsigned int timeout_ms);
#else
static inline bool is_low_temprature(void) { return false; }
static inline int cpufreq_lowtemp_request(const unsigned int timeout_ms) { return 0; }
#endif

#endif /* __HISI_CPUFREQ_LOWTEMP_H__ */
