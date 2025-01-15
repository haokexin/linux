#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <linux/netlink.h>
#include <linux/genetlink.h>
#include <fcntl.h>

#define GTC_GENL_NAME       "BST_GTC_GENL"
#define GTC_GENL_VERSION    1
#define MAX_MSG_SIZE        256
#define GENLMSG_DATA(glh)   ((void *)(NLMSG_DATA(glh) + GENL_HDRLEN))
#define GENLMSG_PAYLOAD(glh)	(NLMSG_PAYLOAD(glh, 0) - GENL_HDRLEN)
#define NLA_DATA(na)        ((void *)((char *)(na) + NLA_HDRLEN))

/* commands */
enum {
    GTC_CMD_UNSPEC,
    GTC_CMD_SYNC_INFO,
    GTC_CMD_USER_INFO,
    __GTC_CMD_MAX,
};
#define GTC_CMD_MAX (__GTC_CMD_MAX - 1)

/* attribute */
enum {
    GTC_ATTR_UNSPEC,
    GTC_ATTR_SYNC_INFO,
    GTC_ATTR_USER_INFO,
    __GTC_ATTR_MAX,
};
#define GTC_ATTR_MAX (__GTC_ATTR_MAX - 1)

typedef struct gtc_msg {
    struct nlmsghdr nlh;
    struct genlmsghdr gnlh;
    char data[MAX_MSG_SIZE];
} gtc_msg_t;

#if 0
typedef struct gtc_sync_info {
    unsigned int sec;
    unsigned int nsec;
} gtc_sync_info_t;
#endif

typedef struct time_sync_parm {
	unsigned int latch_gtc_hicnt;
    unsigned int latch_gtc_lwcnt;
    long long phc_utc_sec;
    long phc_utc_nsec;
    unsigned int gtc_hicnt;
    unsigned int gtc_lwcnt;
} time_sync_parm_t;

typedef struct user_msg {
    char flag;
    unsigned int data;
} user_msg_t;

/**
* send data to kernel by generic netlink
*
* @sock_fd: client socket
* @family_id: family id
* @nlmsg_pid: client pid
* @genl_cmd: cmd type
* @genl_version: genl version
* @nla_type: netlink attr type
* @nla_data: data
* @nla_len: data len
*
* return:
* 0: success; -1: fail
*/
int genl_send_msg(int sock_fd, int16_t family_id, int32_t nlmsg_pid,
        int8_t genl_cmd, int8_t genl_version, int16_t nla_type,
        void *nla_data, int nla_len)
{
    char *buf;
    gtc_msg_t msg;
    struct nlattr *na;
    int ret = -1, buflen;
    struct sockaddr_nl dst_addr;

    /* family id 0 is reserve for ctrl */
    if (family_id == 0) {
        printf("%s family id is invalid\n", __func__);
        return -1;
    }

    /* construct netlink header */
    msg.nlh.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);
    msg.nlh.nlmsg_type = family_id;
    msg.nlh.nlmsg_flags = NLM_F_REQUEST;    //request msg
    msg.nlh.nlmsg_seq = 0;
    msg.nlh.nlmsg_pid = nlmsg_pid;

    /* construct genl netlink header */
    msg.gnlh.cmd = genl_cmd;
    msg.gnlh.version = genl_version;
    na = (struct nlattr *) GENLMSG_DATA(&msg);
    na->nla_type = nla_type;
    na->nla_len = nla_len + 1 + NLA_HDRLEN;
    memcpy(NLA_DATA(na), nla_data, nla_len);
    msg.nlh.nlmsg_len += NLMSG_ALIGN(na->nla_len);
    buf = (char *) &msg;
    buflen = msg.nlh.nlmsg_len;

    /* construct dest addr */
    memset(&dst_addr, 0, sizeof(dst_addr));
    dst_addr.nl_family = AF_NETLINK;
    dst_addr.nl_pid = 0;    //dest pid is 0 (to kernel is 0)
    dst_addr.nl_groups = 0; //unicast

    /* send msg to kernel */
    while ((ret = sendto(sock_fd, buf, buflen, 0, (struct sockaddr *)&dst_addr,
            sizeof(dst_addr))) < buflen) {
        if (ret > 0) {
            buf += ret;
            buflen -= ret;
        } else if (errno != EAGAIN) {
            return -1;
        }
    }

    return 0;
}

/**
* get gtc family id
*
* @sock_fd: client socket
* @family_name: family name registered on kernel
*
* return:
* 0: success; -1: fail
*/
static int get_gtc_family_id(int sock_fd, char *family_name, __u32 *event_group)
{
    gtc_msg_t ans;
    struct nlattr *na;
    struct nlattr *grps;
	struct nlattr *grp;
    int id, ret, rep_len, len;

    ret = genl_send_msg(sock_fd, GENL_ID_CTRL, 0, CTRL_CMD_GETFAMILY, 1,
                    CTRL_ATTR_FAMILY_NAME, (void *)family_name,
                    strlen(family_name) + 1);
    if (ret) {
        printf("%s send genl msg fail\n", __func__);
        return -1;
    }

    rep_len = recv(sock_fd, &ans, sizeof(ans), 0);
    if (rep_len < 0) {
        printf("%s get family id fail\n", __func__);
        return -1;
    }

    if (ans.nlh.nlmsg_type == NLMSG_ERROR || !NLMSG_OK((&ans.nlh), rep_len))
    {
        printf("%s get kernel msg is invalud\n", __func__);
        return -1;
    }

    na = (struct nlattr *)GENLMSG_DATA(&ans);
    //na = (struct nlattr *)((char *)na + NLA_ALIGN(na->nla_len));

    len = 0;
	rep_len = GENLMSG_PAYLOAD(&ans.nlh);
	na = (struct nlattr *)GENLMSG_DATA(&ans);
    while (len < rep_len) {
		len += NLA_ALIGN(na->nla_len);
		if (na->nla_type == CTRL_ATTR_FAMILY_ID) {
			id = *(__u16 *)NLA_DATA(na);
		} else if (na->nla_type == CTRL_ATTR_MCAST_GROUPS) {
			struct nlattr *nested_na;
			struct nlattr *group_na;
			int group_attr_len;
			int group_attr;

			nested_na = (struct nlattr *)((char *)na + NLA_HDRLEN);
			group_na = (struct nlattr *)((char *)nested_na + NLA_HDRLEN);
			group_attr_len = 0;

			for (group_attr = CTRL_ATTR_MCAST_GRP_UNSPEC;
				group_attr < CTRL_ATTR_MCAST_GRP_MAX; group_attr++) {
				if (group_na->nla_type == CTRL_ATTR_MCAST_GRP_ID) {
					*event_group = *(__u32 *)((char *)group_na +
								  NLA_HDRLEN);
					break;
				}

				group_attr_len += NLA_ALIGN(group_na->nla_len) +
						  NLA_HDRLEN;
				if (group_attr_len >= nested_na->nla_len)
					break;

				group_na = (struct nlattr *)((char *)group_na +
							     NLA_ALIGN(group_na->nla_len));
			}
		}
		na = (struct nlattr *)(GENLMSG_DATA(&ans) + len);
	}

    return id;
}

int main(int argc, char* argv[])
{
    struct nlmsgerr *err;
    gtc_msg_t gtc_msg;
    user_msg_t umsg;
    time_sync_parm_t *sync_info;
    struct nlattr *nla;
    struct pollfd pfd;
    struct sockaddr_nl src_addr;
    int ret, len, sock_fd, family_id = 0;
    __u32 group = 0xf;

    /* create a socket */
    sock_fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_GENERIC);
    if (sock_fd < 0) {
        printf("gtc user socket creation fail: %s\n", strerror(errno));
        return -1;
    }

    family_id = get_gtc_family_id(sock_fd, GTC_GENL_NAME, &group);
    if (family_id == 0) {
        printf("get gtc family id fail\n");
        return -3;
    } else {
        printf("gtc family id is %d group id is %d\n", family_id, group);
    }

    /* prepare bind parms */
    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.nl_family = AF_NETLINK;
    src_addr.nl_pid = getpid();
    //src_addr.nl_pid = 12345;
    src_addr.nl_groups = group;

    /* action bind */
    ret = bind(sock_fd, (struct sockaddr *)&src_addr, sizeof(src_addr));
    if (ret < 0) {
        printf("gtc user socket bind fail: %s\n", strerror(errno));
        close(sock_fd);
        return -2;
    }
    
 if (setsockopt(sock_fd, SOL_NETLINK, NETLINK_ADD_MEMBERSHIP,
		       &group, sizeof(group)) < 0)
		printf("could not join the gtc mcast group\n");
    /* send pid to gtc */
    umsg.flag = 1;      //1: pid flag, 0: normal data
    umsg.data = src_addr.nl_pid;

    ret = genl_send_msg(sock_fd, family_id, src_addr.nl_pid, GTC_CMD_USER_INFO, 1, GTC_ATTR_USER_INFO, &umsg, sizeof(umsg));
    if (ret) {
        printf("%s send genl msg fail, ret = %d\n", __func__, ret);
        return -4;
    } else {
        printf("%s send user pid %d success\n", __func__, umsg.data);
    }

    /* set to non-block mode */
    fcntl(sock_fd, F_SETFL, O_NONBLOCK);
    pfd.fd = sock_fd;
    pfd.events = POLLIN; // listen readable events
    pfd.revents = 0;

    while (1) {
        /* use poll to wait for events without occupy the CPU,
        *  -1 means infinite waiting 
        */
        ret = poll(&pfd, 1, -1); 
        if (ret < 0) {
            printf("gtc polling error");
            break;
        } else if (ret == 0) {
            continue;   // timeout handle
        }

        if (pfd.revents & POLLIN) {
            len = recv(sock_fd, &gtc_msg, sizeof(gtc_msg), 0);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    /* no data to read, continue polling */
                    continue;
                }
                printf("gtc genl gtc msg recv failed");
                break;
            } else if (len == 0) {
                printf("connection closed\n");
                break;
            }

            if (gtc_msg.nlh.nlmsg_type == NLMSG_ERROR) {
                printf("get gtc genl nlmsg type is NLMSG_ERROR\n");
                err = (struct nlmsgerr *)NLMSG_DATA(&gtc_msg);
                printf("ERR_NUM=%d - %s\n", err->error, strerror(-err->error));
                break;
            }

            if (gtc_msg.nlh.nlmsg_type == family_id && gtc_msg.gnlh.cmd == GTC_CMD_SYNC_INFO) {
                nla = (struct nlattr *)GENLMSG_DATA(&gtc_msg);
                sync_info = (struct time_sync_parm *)NLA_DATA(nla);
                printf("receive gtc sync info: ");
                printf("latch hicnt=%d lwcnt=%d, phc sec=%lld nsec=%ld, gtc hicnt=%d lwcnt=%d\n",
                    sync_info->latch_gtc_hicnt, sync_info->latch_gtc_lwcnt,
                    sync_info->phc_utc_sec, sync_info->phc_utc_nsec,
                    sync_info->gtc_hicnt, sync_info->gtc_lwcnt);
            }
        }
    }

    close(sock_fd);
    return 0;
}