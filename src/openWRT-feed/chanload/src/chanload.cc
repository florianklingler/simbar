#define ELPP_THREAD_SAFE
#define ELPP_NO_DEFAULT_LOG_FILE

#include "chanload.h"
#include <unistd.h>
#include <string>
#include <iostream>
#include <linux/nl80211.h>
#include <net/if.h>

using namespace std;

// INITIALIZE_EASYLOGGINGPP

int ChannelProber::mNl80211Id;

ChannelProber::ChannelProber() {
	mChannelLoadProbingPeriod = 1000;
	mTotalWifi = 1;
}

ChannelProber::~ChannelProber() {
	//TODO: check what to do here!
	mThreadProbe->join();
}

void ChannelProber::init(const char* ifaceName, int frequency, unsigned int interval, unsigned long start_nsec) {

	mChannelLoadProbingPeriod = interval;	

	int ret;
	wifi = (net_interface*) calloc(mTotalWifi, sizeof(net_interface));
	for (int i = 0; i < mTotalWifi; i++) {
		wifi[i].ifindex = if_nametoindex(ifaceName);
		cerr << "if index: " << wifi[i].ifindex << endl;
		if (wifi[i].ifindex == 0) {
			cerr << "error getting network interface. errno: " << errno << endl;
			wifi[i].ifindex = -1;
			wifi[i].channel = -1;
		} else {
			wifi[i].channel = frequency;
		}
		wifi[i].load.load = -1;
		wifi[i].load.total_time_last = 0;
		wifi[i].load.busy_time_last = 0;
	}

	// initialize socket now
	mSocket = nl_socket_alloc();
	if (mSocket == NULL) {
		cerr << "socket_alloc error" << endl;
		return;
	}
	// disable sequence number checking. We want to receive notifications.
	nl_socket_disable_seq_check(mSocket);

	ret = nl_socket_modify_cb(mSocket, NL_CB_VALID, NL_CB_CUSTOM,
			ChannelProber::receivedNetlinkMsg, this);
	cerr << "NL Socket Callback ret: " << ret << endl;

	// connect socket. Protocol: generic netlink
	ret = genl_connect(mSocket);
	if (ret != 0) {
		cerr << "Connection to Generic Netlink Socket failed! err: "
				<< nl_geterror(ret) << endl;
		return;
	}
	// resolve mNl80211Id
	mNl80211Id = genl_ctrl_resolve(mSocket, "nl80211");
	if (mNl80211Id < 0) {
		cerr << "error code getting nl80211 id: " << nl_geterror(mNl80211Id)
				<< endl;
		return;
	}

	has_time_reached = false;
	start_nsec = start_nsec;

	printf("time\tchanload\n");

	mThreadProbe = new boost::thread(&ChannelProber::probe, this);
}

int ChannelProber::receivedNetlinkMsg(nl_msg *msg, void *arg) {
	// cerr << "Received netlink message!" << endl;
	ChannelProber* cp = (ChannelProber*) arg;

	struct nlattr *tb[NL80211_ATTR_MAX + 1];
	struct genlmsghdr *gnlh = (genlmsghdr*) nlmsg_data(nlmsg_hdr(msg));
	struct nlattr *sinfo[NL80211_SURVEY_INFO_MAX + 1];
	char dev[20];

	uint64_t total_time = 0, busy_time = 0;
	uint32_t channel = 0;
	int8_t noise;

	nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
			genlmsg_attrlen(gnlh, 0), NULL);

	if_indextoname(nla_get_u32(tb[NL80211_ATTR_IFINDEX]), dev);
	// fprintf(stderr, "Survey data from %s\n", dev);

	if (!tb[NL80211_ATTR_SURVEY_INFO]) {
		fprintf(stderr, "survey data missing!\n");
		return NL_SKIP;
	}
	static struct nla_policy survey_policy[NL80211_SURVEY_INFO_MAX + 1] = { };

	if (nla_parse_nested(sinfo, NL80211_SURVEY_INFO_MAX,
			tb[NL80211_ATTR_SURVEY_INFO], survey_policy)) {
		fprintf(stderr, "failed to parse nested attributes!\n");
		return NL_SKIP;
	}

	// If this info is not about the channel in use, then skip
	if (!sinfo[NL80211_SURVEY_INFO_IN_USE]) {
//		cerr << "channel not in use. so returning" << endl;
		return NL_SKIP;
	}
	if (sinfo[NL80211_SURVEY_INFO_FREQUENCY]) {
		channel = nla_get_u32(sinfo[NL80211_SURVEY_INFO_FREQUENCY]);
//		cerr << "\tfrequency:\t\t\t" << channel << " MHz" << endl;
	}
	if (sinfo[NL80211_SURVEY_INFO_NOISE]) {
		noise = (int8_t) nla_get_u8(sinfo[NL80211_SURVEY_INFO_NOISE]);
//		cerr<<"\tnoise:\t\t\t\t"<< noise <<" dBm"<<endl;
	}
	if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME]) {
		total_time = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME]);
//		cerr << "\tchannel active time:\t\t" << total_time << " ms" << endl;
	}
	if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_BUSY]) {
		busy_time = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_BUSY]);
//		cerr << "\tchannel busy time:\t\t" << busy_time << " ms" << endl;
	}
	if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_EXT_BUSY]) {
//		fprintf(stderr, "\textension channel busy time:\t%llu ms\n",
//				(unsigned long long) nla_get_u64(
//						sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_EXT_BUSY]));
	}
	if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_RX]) {
//		cerr << "\tchannel receive time:\t\t"
//				<< nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_RX])
//				<< " ms" << endl;
	}
	if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_TX]) {
//		cerr << "\tchannel transmit time:\t\t"
//				<< nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_TX])
//				<< " ms" << endl;
	}

	// Update stats
	uint64_t busy_time_diff = busy_time - cp->wifi[0].load.busy_time_last;
	uint64_t total_time_diff = total_time - cp->wifi[0].load.total_time_last;
	double load = ((100 * busy_time_diff) / total_time_diff) / 100.0;
	cp->wifi[0].load.mutex_channelLoad.lock();
	cp->wifi[0].load.load = load;
	cp->wifi[0].load.total_time_last = total_time;
	cp->wifi[0].load.busy_time_last = busy_time;
	cp->wifi[0].load.noise = noise;
	cp->wifi[0].load.mutex_channelLoad.unlock();
//	cerr << "time since last check: " << total_time_diff << endl;
//	cerr << "busy time since last check: " << busy_time_diff << endl;
//	cerr << "Calculated Load: " << load << endl;
//	cerr << "-----------------------" << endl;

	if(!cp->has_time_reached) { //continue loop if start_nsec has not been reached yet
			struct timespec systime_ts;
			clock_gettime( CLOCK_REALTIME, &systime_ts);
			long unsigned systime = systime_ts.tv_sec*BILLION + systime_ts.tv_nsec;
			//fprintf(stderr, "curr_time = %lu\n", systime);
			//fprintf(stderr, "start_time = %lu\n", start_nsec);
			if(cp->start_nsec > systime) {
				return NL_SKIP;;
			} else {
				cp->has_time_reached = true;

				struct timeval tv_r;
				gettimeofday(&tv_r, 0);

				cp->startTime = tv_r.tv_sec + (double) (tv_r.tv_usec) / (1000*1000);
			}
	}

	struct timeval tv_r;
	gettimeofday(&tv_r, 0);

	double currTime = tv_r.tv_sec + (double) (tv_r.tv_usec) / (1000*1000);

	//printf("%ld.%06ld\t%f\n", tv_r.tv_sec - cp->start_tv_sec, tv_r.tv_usec - cp->start_tv_usec, load);
	printf("%f\t%f\n", currTime - cp->startTime, load);
	fflush(stdout);

	return NL_SKIP;
}

void ChannelProber::probe() {
	// sleep(5);
	cerr << "started probing every " << mChannelLoadProbingPeriod << " ms" << endl;
	while (true) {
		send_nl80211(NL80211_CMD_GET_SURVEY, &wifi[0].ifindex,
				sizeof(wifi[0].ifindex), NL80211_ATTR_IFINDEX, NL_AUTO_SEQ,
				NLM_F_DUMP);
		nl_recvmsgs_default(mSocket);
		usleep(mChannelLoadProbingPeriod*1000);
	}
}

int ChannelProber::send_nl80211(uint8_t msgCmd, void *payload,
		unsigned int pLength, int payloadType, unsigned int seq, int flags) {
	return send(msgCmd, payload, pLength, payloadType, seq, mNl80211Id, flags,
			0x01);
}

int ChannelProber::send(uint8_t msgCmd, void *payload, unsigned int pLength,
		int attrType, unsigned int seq, int protocol_id, int flags,
		uint8_t protocol_version) {
	int ret = 0;

	// create a new Netlink message
	nl_msg *msg = nlmsg_alloc();

	// create message header
	if (genlmsg_put(msg, 0, seq, protocol_id, 0, flags, msgCmd,
			protocol_version) == NULL) {
//		cerr
//				<< "ChannelProber: Error creating generic netlink header for message."
//				<< endl;
		return -1;
	}

	// add message attributes (=payload)
	if (pLength > 0) {
		ret = nla_put(msg, attrType, pLength, payload);
		if (ret < 0) {
			fprintf(stderr, "ChannelProber: Error message add attribute: %s\n",
					nl_geterror(ret));
			return ret;
		}
	}

	ret = nl_send_auto(mSocket, msg);
	if (ret < 0) {
		fprintf(stderr, "ChannelProber: Error sending message. Error code: %s\n",
				nl_geterror(ret));
	} else {
		//fprintf(stderr, "ChannelProber::send: %d bytes sent\n", ret);
	}
	nlmsg_free(msg);

	return ret;
}

int main(int argc, char *argv[]) {

	if(argc != 5) {
		std::cerr << "usage:" << argv[0] << " <interface> <frequency> <interval in ms> <start_nsec>" << std::endl;
		exit(-1);
	}

	unsigned long start_nsec=(unsigned long)atol(argv[4]);


	ChannelProber prober;
	prober.init(argv[1], atoi(argv[2]), atoi(argv[3]), start_nsec);

	return 0;
}
